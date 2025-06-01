# md25driver_controller

[![GitHub License](https://img.shields.io/github/license/AllJordanSS/md25driver_controller)](https://github.com/AllJordanSS/md25driver_controller/blob/main/LICENSE)

O pacote `md25driver_controller` é um controlador ROS 2 projetado para integrar o hardware MD25 em sistemas robóticos móveis. Ele fornece funcionalidades para controle de motores, monitoramento de hardware e cálculo de odometria, permitindo que o robô seja controlado de forma precisa e eficiente.

Este pacote inclui três nós principais:
- **`motor_controller_node`**: Controla os motores usando um controlador PID.
- **`odometry_node`**: Calcula a odometria do robô com base nos dados dos encoders.
- **`md25_base_controller_node`**: Gerencia a comunicação direta com o hardware MD25.

---

## Principais Funcionalidades

### 1. Controle de Motores com PID
O nó `motor_controller_node` implementa um controlador PID (Proporcional-Integral-Derivativo) para ajustar a velocidade dos motores com base em entradas de referência. O PID é configurável e pode ser ajustado para diferentes cenários de operação.

#### Parâmetros Configuráveis do PID
Os parâmetros do controlador PID podem ser ajustados no arquivo de configuração (`config/hardware.yaml`). Aqui estão os principais parâmetros:

- **`kp`**: Ganho proporcional.
- **`ki`**: Ganho integral.
- **`kd`**: Ganho derivativo.
- **`max_output`**: Valor máximo de saída do controlador.
- **`min_output`**: Valor mínimo de saída do controlador.

Esses parâmetros permitem ajustar o comportamento do controlador para diferentes cargas e condições de operação.

#### Mapeamento de Tensão
O nó também mapeia os valores de tensão mínima e máxima suportados pelo hardware MD25. Isso garante que os sinais enviados aos motores estejam dentro da faixa segura de operação.

---

### 2. Configuração de Hardware
O nó `motor_controller_node` permite configurar parâmetros relacionados ao hardware do robô. Esses parâmetros são essenciais para garantir que o controle de motores e a odometria sejam calculados corretamente.

#### Parâmetros Configuráveis
- **`wheel_diameter`**: Diâmetro das rodas do robô (em metros).
- **`encoder_cpr`**: Contagem de pulsos por revolução (CPR) dos encoders.
- **`gear_ratio`**: Razão de redução entre o motor e as rodas.

Esses parâmetros devem ser ajustados no arquivo de configuração (`config/hardware.yaml`) para corresponder às especificações do seu robô.

---

### 3. Cálculo de Odometria
O nó `odometry_node` calcula a odometria do robô com base nos dados dos encoders. Ele publica mensagens no tópico `/odom` no formato padrão ROS 2 (`nav_msgs/Odometry`).

#### Saída de Odometria
A odometria inclui:
- Posição (`x`, `y`) e orientação (`theta`) do robô no espaço.
- Velocidades linear e angular.

Essas informações podem ser usadas para navegação autônoma ou visualização em ferramentas como RViz.

---

## Instalação e Uso

### 1. Pré-requisitos
Certifique-se de que o ROS 2 Humble ou superior esteja instalado no seu sistema. Você também precisará do hardware MD25 conectado ao seu robô ou computador via I2C.

### 2. Clonar o Repositório
Clone o repositório para o seu workspace ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/AllJordanSS/md25driver_controller.git
```

### 3. Compilar o Pacote
Compile o pacote usando `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select md25driver_controller
source install/setup.bash
```

### 4. Executar os Nós
Você pode iniciar os nós individualmente ou usar o arquivo de lançamento fornecido.

#### Execução Individual
```bash
ros2 run md25driver_controller motor_controller_node
ros2 run md25driver_controller odometry_node
ros2 run md25driver_controller md25_base_controller_node
```

#### Usando o Arquivo de Lançamento
```bash
ros2 launch md25driver_controller motor_controler.launch.py
```
---
### 5. **Interação com o Pacote**

O pacote `md25driver_controller` fornece uma interface ROS 2 padrão para interagir com o hardware MD25 e controlar seu robô. Abaixo estão descritos os principais tópicos e como configurar o controlador PID para sua base robótica.

### **Publicar Comandos de Velocidade (`/cmd_vel`)**
Para controlar a velocidade do robô, publique mensagens no tópico `/cmd_vel`. Esse tópico aceita mensagens do tipo `geometry_msgs/Twist`, que contêm informações de velocidade linear e angular.

Exemplo de comando para enviar um comando de movimento:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

- **`linear.x`**: Velocidade linear na direção frontal (em m/s).
- **`angular.z`**: Velocidade angular em torno do eixo vertical (em rad/s).

### **Ler Odometria (`/wheels_odom`)**
A odometria calculada pelo nó `odometry_node` é publicada no tópico `/wheels_odom`. Esse tópico fornece mensagens no formato `nav_msgs/Odometry`, que incluem a posição (`x`, `y`, `theta`) e as velocidades linear e angular do robô.

Para visualizar a odometria em tempo real, use o seguinte comando:

```bash
ros2 topic echo /wheels_odom
```

Essas informações podem ser usadas para navegação autônoma ou visualização em ferramentas como RViz.

### **Ler Dados dos Encoders (`/md25_encoders`)**
Os dados dos encoders são publicados no tópico `/md25_encoders`. Esse tópico fornece mensagens personalizadas do tipo `md25_controller/msg/Md25Encoders`, que incluem informações como:

- Posição dos encoders (`encoder_l`, `encoder_r`).
- Velocidade dos motores (`left_vel_rpm`, `right_vel_rpm`).
- Variação de pulsos (`left_delta_pulses`, `right_delta_pulses`).

Para visualizar os dados dos encoders em tempo real, use o seguinte comando:

```bash
ros2 topic echo /md25_encoders
```

Aqui está a seção de **Configuração** atualizada e organizada para cópia direta. Incluí detalhes sobre como configurar cada nó com base no arquivo `params.yaml` fornecido:

---

### **6. Configuração**

#### **6.1 Arquivo de Configuração**
O pacote `md25driver_controller` utiliza arquivos YAML (`config/param.yaml`) para configurar os parâmetros dos nós. Abaixo estão os principais parâmetros e suas descrições.

---

#### **6.2 Configuração do Nó `motor_controller_node`
Este nó é responsável pelo controle de motores usando um controlador PID. Ele também mapeia os tópicos relacionados aos encoders e comandos de velocidade.

##### **Parâmetros Configuráveis**
```yaml
motor_controller:
  ros__parameters:
    traction_control:
      control_rate: 50                     # Frequência de controle (Hz)
      pid_params_left:                     # Parâmetros PID para o motor esquerdo
        P: 5.0                             # Ganho proporcional
        I: 10.0                            # Ganho integral
        D: 0.0                             # Ganho derivativo
        lBounds: -12.0                     # Limite inferior de saída (tensão mínima)
        hBounds: 12.0                      # Limite superior de saída (tensão máxima)
        windupGuard: 0.15                  # Proteção contra windup (acumulação excessiva do termo integral)
      pid_params_right:                    # Parâmetros PID para o motor direito
        P: 5.0                             # Ganho proporcional
        I: 10.0                            # Ganho integral
        D: 0.0                             # Ganho derivativo
        lBounds: -12.0                     # Limite inferior de saída (tensão mínima)
        hBounds: 12.0                      # Limite superior de saída (tensão máxima)
        windupGuard: 0.15                  # Proteção contra windup
    traction_general:
      encoders_topic: "/md25_encoders"     # Tópico para ler os dados dos encoders
      cmd_vel_topic: "/cmd_vel"            # Tópico para receber comandos de velocidade
      md25_cmd_topic: "/md25_voltage_commands" # Tópico para enviar comandos de tensão ao MD25
      wheel_radius: 0.0325                 # Raio das rodas (em metros)
      wheels_separation: 0.18              # Distância entre as rodas (em metros)
```

##### **Dicas para Ajuste do PID**
- **Ganho Proporcional (`P`)**: Controla a resposta imediata ao erro. Um valor muito alto pode causar oscilações, enquanto um valor muito baixo pode resultar em uma resposta lenta.
- **Ganho Integral (`I`)**: Corrige erros acumulados ao longo do tempo. Use com cuidado para evitar instabilidade.
- **Ganho Derivativo (`D`)**: Suaviza a resposta e reduz oscilações. Geralmente é usado em sistemas com alta inércia.

Certifique-se de testar diferentes valores para encontrar a configuração ideal para sua base robótica.

---

#### **6.3 Configuração do Nó `odometry_node`
Este nó calcula a odometria do robô com base nos dados dos encoders.

##### **Parâmetros Configuráveis**
```yaml
wheels_odometry:
  ros__parameters:
    traction_general:
      rate: 10                            # Frequência de publicação da odometria (Hz)
      encoders_topic: "/md25_encoders"    # Tópico para ler os dados dos encoders
      odom_camera_topic: "/camera/odom/sample" # Tópico opcional para fusão de odometria visual
      odom_topic: "/wheels_odom"          # Tópico para publicar a odometria calculada
      wheel_radius: 0.0325                # Raio das rodas (em metros)
      wheels_separation: 0.18             # Distância entre as rodas (em metros)
      linearGain: 10.0                    # Ganho para cálculo de velocidade linear
      angularGain: 40.0                   # Ganho para cálculo de velocidade angular
```

##### **Dicas para Configuração**
- Certifique-se de que os valores de `wheel_radius` e `wheels_separation` correspondam às dimensões físicas do seu robô.
- O ajuste dos ganhos `linearGain` e `angularGain` pode ser necessário para melhorar a precisão da odometria.

---

#### **6.4 Configuração do Nó `md25_base_controller_node`
Este nó gerencia a comunicação direta com o hardware MD25.

##### **Parâmetros Configuráveis**
```yaml
md25_base_controller:
  ros__parameters:
    md25:
      i2c_bus: "/dev/i2c-3"               # Barramento I2C utilizado
      i2c_address: 0x58                   # Endereço I2C do MD25
      regulator: true                     # Habilita/desabilita o regulador de tensão
      timeout: true                       # Habilita/desabilita o temporizador de segurança
      acquisition_rate: 50                # Frequência de leitura dos dados do MD25 (Hz)
      publish_topics:
        encoders: "md25_encoders"         # Tópico para publicar os dados dos encoders
        data: "md25_data"                 # Tópico para publicar os dados gerais do MD25
      subscribe_topics:
        commands: "md25_voltage_commands" # Tópico para receber comandos de tensão
        reset_encoders: "md25/reset"      # Tópico para resetar os encoders
    encoders:
      cpr: 48.0                           # Contagem de pulsos por revolução (CPR) dos encoders
      motorRatio: 46.85                   # Razão de redução entre o motor e as rodas
```

##### **Dicas para Configuração**
- Verifique se o barramento I2C (`i2c_bus`) e o endereço (`i2c_address`) estão corretos para o seu hardware.
- Ajuste o valor de `cpr` e `motorRatio` para corresponder às especificações dos encoders e do sistema de transmissão.

---

### **7. Resumo dos Parâmetros**
| Parâmetro                | Descrição                                      | Exemplo          |
|--------------------------|------------------------------------------------|------------------|
| `control_rate`           | Frequência de controle do PID (Hz)             | `50`             |
| `P`, `I`, `D`            | Ganhos do controlador PID                      | `5.0, 10.0, 0.0` |
| `wheel_radius`           | Raio das rodas (m)                             | `0.0325`         |
| `wheels_separation`      | Distância entre as rodas (m)                   | `0.18`           |
| `encoders_topic`         | Tópico para ler os dados dos encoders          | `/md25_encoders` |
| `cmd_vel_topic`          | Tópico para receber comandos de velocidade     | `/cmd_vel`       |
| `odom_topic`             | Tópico para publicar a odometria               | `/wheels_odom`   |
| `i2c_bus`                | Barramento I2C utilizado                       | `/dev/i2c-3`     |
| `i2c_address`            | Endereço I2C do MD25                           | `0x58`           |
| `cpr`                    | Contagem de pulsos por revolução dos encoders  | `48.0`           |
| `motorRatio`             | Razão de redução entre motor e rodas           | `46.85`          |

---

Certifique-se de ajustar os parâmetros conforme as especificações do seu robô e hardware MD25.

---

## Contribuições
Contribuições são bem-vindas! Se você encontrar problemas ou quiser adicionar novas funcionalidades, abra uma issue ou envie um pull request.

---

