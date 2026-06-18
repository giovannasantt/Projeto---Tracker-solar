#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <pwm_z42.h>
#include <math.h>

#define PI 3.14159265358979323846
#define RAD(deg) ((deg) * PI / 180.0)
#define DEG(rad) ((rad) * 180.0 / PI)

// Configuração do período para 50Hz usando Clock de 48MHz e Prescaler 128
#define TPM_MODULE     7500  

// Valores de contagem para o registrador CnV (Duty Cycle do Servo)
#define MIN_DUTY_CNV   375   // 1ms de pulso (0 graus)
#define MAX_DUTY_CNV   750   // 2ms de pulso (180 graus)

// Canais e Pinos Físicos (Exemplo: Usando portas GPIOB 18 e 19 no módulo TPM2)
#define CANAL_ELEVACAO 0
#define PINO_ELEVACAO  18

#define CANAL_AZIMUTE  1
#define PINO_AZIMUTE   19

typedef struct {
    double elevacao; 
    double azimute;  
} PosicaoSolar;

// --- FUNÇÕES DE CÁLCULO SOLAR ---
int calcular_n(int dia, int mes) {
    int dias_mes[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int n = dia;
    for (int i = 1; i < mes; i++) n += dias_mes[i];
    return n;
}

double calcular_declinacao(int n) {
    return 23.45 * sin(RAD(360.0 / 365.0 * (284 + n)));
}

double calcular_omega(double hl, int n, double long_local, double long_ref) {
    double B = RAD((360.0 / 364.0) * (n - 81));
    double E = 9.87 * sin(2 * B) - 7.53 * cos(B) - 1.5 * sin(B);
    double cor_hora = (4.0 * (long_ref - long_local) + E) / 60.0;
    return 15.0 * (hl + cor_hora - 12.0);
}

PosicaoSolar calcular_posicao_solar(double latitude, int n, double hl, double long_local, double long_ref) {
    double delta = RAD(calcular_declinacao(n));
    double phi = RAD(latitude);
    double w = RAD(calcular_omega(hl, n, long_local, long_ref));
    
    PosicaoSolar sol;
    double sin_a = sin(delta) * sin(phi) + cos(delta) * cos(phi) * cos(w);
    sol.elevacao = DEG(asin(sin_a));
    
    double cos_gs = (sin_a * sin(phi) - sin(delta)) / (cos(asin(sin_a)) * cos(phi));
    sol.azimute = DEG(acos(cos_gs));
    if (w > 0) sol.azimute = 360.0 - sol.azimute;
    
    return sol;
}

// --- CONTROLE DOS SERVOS (BIBLIOTECA PERSONALIZADA) ---
void mover_tracker(double angulo_e, double angulo_a) {
    // Proteção de limites físicos (0 a 180 graus)
    if (angulo_e < 0) angulo_e = 0; if (angulo_e > 180) angulo_e = 180;
    if (angulo_a < 0) angulo_a = 0; if (angulo_a > 180) angulo_a = 180;

    // Converte os ângulos proporcionalmente para a faixa do registrador CnV (375 a 750)
    uint16_t cnv_elevacao = MIN_DUTY_CNV + (uint16_t)((angulo_e / 180.0) * (MAX_DUTY_CNV - MIN_DUTY_CNV));
    uint16_t cnv_azimute  = MIN_DUTY_CNV + (uint16_t)((angulo_a / 180.0) * (MAX_DUTY_CNV - MIN_DUTY_CNV));

    // Atualiza diretamente os registradores dos canais correspondentes
    pwm_tpm_CnV(TPM2, CANAL_ELEVACAO, cnv_elevacao);
    pwm_tpm_CnV(TPM2, CANAL_AZIMUTE, cnv_azimute);
}

int main(void)
{
    // Inicializa o módulo TPM2 com o valor de MOD em 7500 (Garante os 50Hz exigidos pelos servos)
    pwm_tpm_Init(TPM2, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);

    // Inicializa o canal 0 para o Servo de Elevação na porta GPIOB_18
    pwm_tpm_Ch_Init(TPM2, CANAL_ELEVACAO, TPM_PWM_H, GPIOB, PINO_ELEVACAO);

    // Inicializa o canal 1 para o Servo de Azimute na porta GPIOB_19
    pwm_tpm_Ch_Init(TPM2, CANAL_AZIMUTE, TPM_PWM_H, GPIOB, PINO_AZIMUTE);

    // Configurações Geográficas (São Paulo)
    double lat = -23.57;
    double long_local = 46.73;
    double long_ref = 45.0; 

    // Loop infinito de rastreamento
    for (;;)
    {
        // Variáveis de tempo (Exemplo fixo para testes, ex: 18 de Junho às 14:30)
        int dia = 18, mes = 6;
        double hora = 14.5; 

        // Executa o modelo matemático solar
        int n = calcular_n(dia, mes);
        PosicaoSolar sol = calcular_posicao_solar(lat, n, hora, long_local, long_ref);

        // Move os motores gerando os pulsos corretos nos registradores
        mover_tracker(sol.elevacao, sol.azimute);

        // Aguarda 1 minuto (60000 milissegundos) antes do próximo ajuste
        k_msleep(60000);
    }

    return 0;
}