## TECHMAKER ROBOTICS 2022

# Projeto
Projeto do Robo da Techmaker Robotics 2022 - Rapid React
## Itens

- [Projeto](#projeto)
  - [Itens](#itens)
  - [Rede CAN <a name = "CAN"></a>](#rede-can-)
  - [Sensores <a name = "Sensors"></a>](#sensores-)
  - [Controles <a name = "controls"></a>](#controles-)

## Rede CAN <a name = "CAN"></a>

**VICTOR SPX**
|Endereço|Função|
|-|-|
|0|Intake|
|1|Mira (aim)|
|2|Climber|
|3|Conveyor|
|4|Shooter Left|
|5|Shooter Right|
|6|Drive Left|
|7|Drive Right|
|8|Drive Left|
|9|Drive Right|

**Spark Max**
|Endereço|Função|
|-|-|
|0|Tiro (trigger)|

## Sensores <a name = "Sensors"></a>

**ENCODERS**
|A|B|Função|
|-|-|-|
|0|1|Roda direita|
|2|3|Roda esquerda|
|4|5|Shooter|
|6|7|Mira|

**Cor**

Via Serial com Arduino

## Controles <a name = "controls"></a>

**Controle XBOX**

Somente navegação

**Controle personalizado**

*Disposição dos botões*
| |_C1_|_C2_|_C3_|_C4_|
|-|-|-|-|-|
|**_L1_**|9|10|11|12|
|**_L2_**|5|6|7|8|
|**_L3_**|1|2|3|4|
  
|Botão|Funções|
|-|-|
|1|***Shooter aim***|
| |Faz leitura da Lumilight|
| |ajusta Mira|
| |Ajusta rotação do shooter|
|-|-|
|2|***Trigger***|
| |Liga motor trigger|
| |Liga motor conveyor|
|-----|--------------------------|
|5|***Conveyor***|
| |Liga motor Conveyor|
| |Liga Motor Intake|
|-----|--------------------------|
|6|***Conveyor Out***|
| |Reverte motor Conveyor|
| |Reverte motor Intake|
|-----|--------------------------|
|8|***Climber Revert***|
| |Reverte o motor do climber, para calibrar inicio de partida|
|-----|--------------------------|
|9|***Intake***|
| |Ativa valvula para descer intake|
| |Liga motor Intake|
| |Liga motor Conveyor|
|-----|--------------------------|
|12|***Climber***|
|  |Liga motor do climber|
