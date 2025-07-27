Projeto: TRENA – Medidor de Distância
Este projeto implementa uma trena digital utilizando um sensor óptico capaz de medir distâncias de até 1200 mm, com comunicação serial via UART. O sistema foi desenvolvido para rodar em um microcontrolador ESP32 e pode ser visualizado via terminal serial.

⚠️ Observações Importantes:

O display I2C foi desabilitado neste projeto devido a conflitos no barramento de comunicação.

Durante os testes, foram observados maus contatos quando encostávamos na bancada, o que afetava momentaneamente o funcionamento do sistema.

🧰 Componentes Utilizados
ESP32 DevKit

Sensor óptico de distância (VL53L0X ou similar)

Comunicação UART (via cabo USB)

Terminal serial (PuTTY, minicom, etc)

Jumpers e protoboard

🔧 Como funciona
O sensor óptico mede a distância até um obstáculo.

O valor é enviado para o computador via UART.

A medição é exibida no terminal serial.

⚙️ Configuração UART
Parâmetro	Valor
Baud rate	115200
Bits de dados	8
Paridade	Nenhuma
Bits de parada	1
UART utilizada	UART0

🛠️ Como compilar e executar
Antes de compilar, selecione o chip correto:

bash
Copiar
Editar
idf.py set-target esp32
Para compilar, gravar e monitorar a saída serial:

bash
Copiar
Editar
idf.py -p PORT flash monitor
(Substitua PORT pela porta USB correta do seu sistema. Para sair do monitor, use Ctrl + ].)

🖥️ Exemplo de saída no terminal
makefile
Copiar
Editar
Medindo distância...
Distância: 455 mm

Medindo distância...
Distância: 612 mm
🚧 Problemas Conhecidos
O display I2C foi desabilitado por conflitos no barramento.

Maus contatos foram percebidos ao tocar na bancada durante os testes.

Certifique-se de que os cabos estejam firmes para evitar falhas de comunicação ou leitura.

📚 Referências
Datasheet do sensor óptico utilizado

Documentação do ESP-IDF
Link: https://components.espressif.com/components?q=Vl53l0x

