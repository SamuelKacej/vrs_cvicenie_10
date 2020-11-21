# Náplň cvičenia
- zoznámenie sa s časovačom (TIM)


# Konfigurácia časovača (TIM3)

<p align="center">
    <img src="https://github.com/VRS-Predmet/vrs_cvicenie_10/blob/master/images/tim_config.PNG" width="800">
</p>

- timer 3 využiva interný zdroj hodín čo je v tomto prípade 8MHz

- v projekte je TIM3 využitý na generovanie prerušenia v pravidelnom časovom intervale a preto je dôležité správne nastavenie počítadla a predeličky (prescaler)

- predelička nastavená na 7999 znamená, že počítadlo časovača bude inkrementované každú milisekundu (znížil som frekvenciu inkrementovania počítadla z 8MHz na 1kHz)

- časovač nastavený na 999 znamená, že ak počítadlo napočíta do 999, nastane prerušenie (update event) a počítadlo začne počítať od 0 (ak počítame nahor - up-counting)

### PWM - pulse width modulation

<p align="center">
    <img src="https://github.com/VRS-Predmet/vrs_cvicenie_10/blob/zadanie_cv10/images/pwm-output-mode.jpg" width="700">
</p>

- Duty cycle (strieda) - pomer dĺžky trvania aktívneho stavu signálu a jeho dĺžky periódy
- pre nastavenie "duty cycle" slúži capture/compare register (TIMx_CCRx)

- pulse_length = ((TIMx_ARR) * DutyCycle) / 100  (hodnota zapisovaná do TIMx_CCRx)

- Balik obsahujuci knižnice, middleware, príklady ... : https://www.st.com/en/embedded-software/stm32cubef3.html


# Zadanie
Vytvorte frimware pre MCU, ktorý bude vypínať a zapínať LED spôsobom pripomínajúcim "fade in" a "fade out" efekt. Program bude mať dva módy - manuálny a automatický. V manuálnom móde bude LED plynule a autoamticky prechádzať medzi stavmi "ON" a "OFF". To znamená, že ak je LED na začiatku v stave "ON", tak sa intenzita jej svietenia začne pomaly zmenšovať až do momentu, pokiaľ nezhasne. Keď LED zhasne (teda je v stave OFF), tak sa začne postupne rozsvecovať. V manuálnom móde bude možné nastavovať intenzitu svietenia LED prostredníctvom terminálu z PC pričom prechod z jednej intenzity svietenia na druhú musí byť plynulý. Prepínanie medzi manuálnym a automatickým módom bude taktiež ovládateľne prostredníctvom terminálu z PC.

### Úlohy
1. Vytvoriť nový projekt, v ktorom nakonfigurujete periférie nevyhnutné pre toto zadanie.

2. Nakonfigurovať perifériu USART2 tak, aby bola umožnená komunikácia medzi PC a MCU. Pre nastavenie USART2 využite rovnaké hodnoty parametrov (baud rate, length, stop bits ...) ako z predošlého zadania (zadanie_cv7). Nakonfigurujte DMA tak, aby bola využívaná perifériou USART2 na príjem a odosielanie dát. Príjem a odosielanie dát budú obslúžené v prerušení, tak ako to bolo v predošlom zadaní (zadanie_cv7).

3. Pre prepínanie medzi manuálnym a automatickým módom budú slúžiť príkazy "manual" a "auto". Na to, aby bol príkaz zo strany MCU prijatý sa musí začínať a končiť znakom "$".  V manuálnom režime bude pre nastavenie intezity svietenia LED slúžiť príkaz "PWMxx", kde "xx" predstavuje číslo v rozsahu 0 - 99. Napríklad "PWM05" alebo "PWM48". Taktiež platí, že sa príkaz musi začínať a končiť znakom "$". Ak chcem napríklad v manuálnom režime nastaviť, aby LED svietila na 50%, tak najprv bude nutné z termínálu zaslať príkaz "$manual$" a následne zaslať príkaz "$PWM50$".

4. Nakonfigurovať časovač TIM2 tak, aby generoval PWM signál, ktorý bude vyvedený na GPIOA-5. Vyberte vhodný kanál časovača, ktorý je na daný pin vyvedený. Predeličku a počítadlo časovača nastavte tak, aby frekvencia PWM signálu bola 100Hz.

5. Vytvoriť funkciu, pomocou ktorej bude možné meniť šírku PWM signálu počas behu programu. Deklarácia funkcie bude vyzerať následovne - "void setDutyCycle(uint8_t D)". Funkcia nemá žiadnu návratovú hodnotu. Vstupným parametrom je číslo, duty cycle, v rozpätí 0 - 99.

6. Nakonfigurovať časovač TIM 2 tak, aby dokázal generovať prerušenie v pravidelnom časovom intervale. V obsluhe prerušenia od časovača sa bude aktualizovať hodnota šírky PWM signálu pomocou funkcie z bodu 6. Vyberte vhodný kanál časovača pre daný účel. Predeličku a počítadlo časovača nastavte tak, aby sa prerušenie generovalo každých 10ms. 

7. LED, ktorej intenzita svietenia sa bude ovládať, bude pripojená ku GPIOA-5, na ktorý je vyvedený PWM signál od časovača TIM2.

8. Automatický režim - LED sa počas jednej sekundy plynulo dostane zo stavu "ON" do stavu "OFF". Počas nasledujúcej sekundy sa plynulo dostane zo stavu "OFF" do stavu "ON". To znamená, že LED sa bude postupne rozsvecovať a následne postupne zhasovať. Perióda blikania LED je teda 2s -> 1s na zhasnutie a 1s na rozsvietenie LED.

9. Manuálny režim - Intenzita svietenia LED je ovládaná prikazmi z PC. Prechod medzi novou žiadanou intenzitou a aktuálnou intenzitou svietenia musí byť plynulý. Čas pre rozsvietenie LED z 0% na 100% alebo zhasnutie zo 100% na 0% je 1s. To znamená, že ak je intenzita svietenia LED aktuálne nastavená na 50% a nová žiadaná hodnota bude maximálna intezita svietenia (100%), tak plynulý prechod bude trvať 500ms. Z vopred predpísaných hodnôť vychádza, že sa intenzita svietenia mení zo skokom 1% (v manuálnom aj v automatickom režime).
