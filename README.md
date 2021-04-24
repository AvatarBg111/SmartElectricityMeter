# SmartElectricityMeter

# Описание:
 SmartElectricityMeter е смарт електромер, който ще Ви помогне да следите в реално време и да пестите Вашата консумация на електричество.  
С изградения интерфейс на уебсайта "Smart Electricity Meter" ще можете да комуникирате с Вашия електромер, който има функцията да разпознава употребяваните от Вас консуматори, използвайки невронна мрежа.

## Планове и бъдещи идеи:
* Да се реализира невронна мрежа, която да разпознава работещите консуматори в електрическата система на smart дома
* Да се изградят "по-приличен" и интерактивен интерфейс и облик на сайта за удобство и леснота на работа за потребителя
* Да се имплементират уебприложение и андроид приложение
* Да се вгради функционираща файлова система (littlefs) за запаметяване на специални измервания, получени от чип MCP32F511, които да се използват за обучение на невронна мрежа 

## Използвани технологии:
#### Online Website
* [CSS](https://www.w3.org/Style/CSS/Overview.en.html)
* [D3](https://d3js.org/d3.v3.min.js) (library direct link)
* [HTML](https://html.com/)
* [JavaScript](https://www.javascript.com/)
* [jQuery](https://jquery.com/)
* [PHP](https://www.php.net/)
#### Hardware
* [MCP39F511](http://ww1.microchip.com/downloads/en/DeviceDoc/20005393B.pdf)
* [STM32F407](https://www.st.com/en/microcontrollers-microprocessors/stm32f407-417.html)
* [ESP32](https://www.espressif.com/en/products/socs/esp32)
* [C-The-Lang](https://port70.net/~nsz/c/c11/n1570.html)

## Изисквания и нужни билбиотеки за Хардуер
#### ESP32-WROOM-32D
Espressif IoT Development Framework
```
git clone https://github.com/espressif/esp-idf
```

#### STM32F407
STM32F4 Standard Periphery library and CMSIS library
```
git clone https://github.com/mfauzi/STM32F4/tree/master/STM32F4%20Standard%20Peripheral%20Library
```
## Упътвания за използване на сайт
Уебсайтът е в ранна версия с лимитирани функции.
За момента има качествени измервания само за 
12.04.2021 (изберете тази дата, за да видите графика)
#### [Link: SmartElectricityMeter](http://learningmoorree.000webhostapp.com/)

#### Документация на проекта (снимки, схеми и т.н.) се намира в папка Doc
