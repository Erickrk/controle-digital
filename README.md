[![LinkedIn][linkedin-shield]][linkedin-url]
# Controle_Digital
Repositório com códigos da aplicação desenvolvida em C++ para a disciplina de Controle Digital do CEFET-RJ, unidade Maracanã. Um diagrama geral do projeto está disponível abaixo:

<p align="center">
  <img src="https://github.com/Erickrk/Controle_Digital/blob/main/images/Projeto%20Digital.png" alt="Diagrama do Projeto"/>
</p>

O objetivo proposto foi construir uma solução capaz de fazer a telemetria e controle de um motor elétrico. Para esse fim foram utilizados o protocolo de comunicação MQTT e um algoritmo de controle PID. Adicionalmente, foi utilizado o InfluxDB para gestão do banco de dados e criação do dashboard.


## Componentes utilizados:
- Dois motores, usados normalmente em kits de robótica (os amarelos);
- Um módulo de Ponte H com a L298;
- Um encoder óptico;
- Um disco para encoder óptico de 30 rasgos;
- Um Esp32;
- Uma fonte de 5V para protoboard; e
- Duas protoboards e jumpers.


O desenvolvimento foi feito no plugin PlatformIO, o que explica a disposição interna das pastas. As bibliotecas que não estão disponíveis pelo gerenciador do PlatformIO têm o link para o Github no código.


## Print da aplicação funcionando:
<p align="center">
  <img src="https://github.com/Erickrk/Controle_Digital/blob/main/images/app.png" alt="Aplicação"/>
</p>


## Este repositório contém:
- Encoder4LinReg: utililzado para comparar a rotação medida pelo ESP, através do encoder, e os parâmetros passados para a função de movimento. Sua finalidade é auxiliar na coleta de dados que podem ser usados em uma regressão linear.
- MotorControl: foi feito sem todas as funções de conectividade (WI-FI e MQTT a fim de conhecer melhor o sistema e os parâmetros PID do mesmo.
- MotorControl_mqtt: código final apresentado na disciplina. Conta com a implementação da WI-FI, MQTT e PID no Esp32.
- Server_side: aplicação que roda no lado do servidor. Se comunica com o Broker MQTT e integra ao InfluxDB.


## Algumas sugestões de melhoria no projeto:
- Mudança dos tipos de variável para uintx_t;
- Fazer um estudo mais aprofundado sobre o derivativo;
- Realizar integração com o MATLAB, pode ser interessante para ter um Observador;
- Construir bibliotecas para deixar o código mais modular;
- Implementar FreeRTOS, talvez separar um core para conectividade e outro para controle;
- Separar os códigos de conectividade e controle em dois microcontrolaroes separados, se comunicando; e
- Motores de maior qualidade podem ter respostas mais agradáveis, pode ser interessante testar modelos diferentes. 

## Agradecimentos: 
A realização desse trabalho foi muito facilitada graças ao esforço do [Igor Kelvin](https://github.com/igorkelvin), com apoio à definição do escopo e requisitos do projeto, auxilio na parametrização do algoritmo de PID e na implementação de filtros digitais. Também, a ajuda do Isvaldo Ferndandes, CTO da @Squair, essencial para a aplicação da tecnologia do InfluxDB nessa solução.
Agradeço imensamente aos dois, assim como a todos os responsáveis pelo conteúdo presente na seção de Referências.


## Referências:
- [Playlist de vídeos de outra solução de IoT com tecnologias diferentes (Node Red e IBM Watson)](https://www.youtube.com/watch?v=T-Xg15Iokhg&list=PLSfFtg91FAVEDSOPn_OqHtFy3zzFNB2Q-)
- [Bíblia do Esp32](https://leanpub.com/kolban-ESP32)
- [Código da aplicação Web feita pela turma de Controle Digital de 2019.2](https://github.com/igorkelvin/controle-digital-de-motor-dc)
- [Código feito pela turma de Controle Digital de 2019.1](https://github.com/piradata/motor_control)
- [Interrupções externas no Esp32](https://diyprojects.io/esp32-how-use-external-interrupts-arduino-code/#.X6MfJ6vPyCo)
- [Explanação geral sobre encoders ópticos](https://www.usinainfo.com.br/blog/sensor-de-velocidade-arduino-medindo-a-rotacao-de-motores/)
- [Tutorial básico de PID em Arduino](https://www.arrow.com/en/research-and-events/articles/pid-controller-basics-and-tutorial-pid-implementation-in-arduino)
- [Fonte principal do algoritmo de Controle aplicado](https://www.element14.com/community/community/arduino/blog/2020/01/06/simple-arduino-dc-motor-control-with-encoder-part-2)
- [Vídeo que explica alguns conceitos de PID na prática](https://www.youtube.com/watch?v=AN3yxIBAxTA)
- [Conceito bem próximo da aplicação apresentada aqui](https://www.youtube.com/watch?v=QSIksPKndEs)


[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/silva-erick/

