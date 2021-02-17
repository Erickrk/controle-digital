# Controle_Digital
Repositório com códigos da aplicação desenvolvida em C++ para a disciplina de Controle Digital do CEFET-RJ, unidade Maracanã. Um diagrama geral do projeto está disponível em: https://drive.google.com/file/d/1NI6kZV08NlMY5wC1NHvddC2JuwzCzkOu/view?usp=sharing 
O objetivo proposto foi construir uma solução capaz de fazer a telemetria e controle de um motor elétrico. Para esse fim foram utilizados o protocolo de comunicação MQTT e um algoritmo de controle PID. Adicionalmente, foi utilizado o InfluxDB para gestão do banco de dados e criação do dashboard.


- Componentes utilizados:
. Dois motores, usados normalmente em kits de robótica (os amarelos);
. Um módulo de Ponte H com a L298;
. Um encoder óptico;
. Um disco para encoder óptico de 30 rasgos;
. Um Esp32;
. Uma fonte de 5V para protoboard; e
. Duas protoboards e jumpers.


O desenvolvimento foi feito no plugin PlatformIO, o que explica a disposição interna das pastas. As bibliotecas que não estão disponíveis pelo gerenciador do PlatformIO têm o link para o Github no código.


- Este repositório contém:
. Encoder4LinReg: utililzado para comparar a rotação medida pelo ESP, através do encoder, e os parâmetros passados para a função de movimento. Sua finalidade é auxiliar na coleta de dados que podem ser usados em uma regressão linear.
. MotorControl: foi feito sem todas as funções de conectividade (WI-FI e MQTT a fim de conhecer melhor o sistema e os parâmetros PID do mesmo.
. MotorControl_mqtt: código final apresentado na disciplina. Conta com a implementação da WI-FI, MQTT e PID no Esp32.


- Algumas sugestões de melhoria no projeto:
. Mudança dos tipos de variável para uintx_t;
. Fazer um estudo mais aprofundado sobre o derivativo;
. Realizar integração com o MATLAB, pode ser interessante para ter um Observador;
. Construir bibliotecas para deixar o código mais modular;
. Implementar FreeRTOS, talvez separar um core para conectividade e outro para controle;
. Separar os códigos de conectividade e controle em dois microcontrolaroes separados, se comunicando; e
. Motores de maior qualidade podem ter respostas mais agradáveis, pode ser interessante testar modelos diferentes. 
