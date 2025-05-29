# ble-synchronization

## Documentation

Pinout i pojašnjenje svakog pina: https://mischianti.org/esp32-nodemcu-32s-esp-32s-kit-high-resolution-pinout-datasheet-and-specs/

## Dev kits

Na raspolaganju imamo dvije vrste, jedna je sa micro usb priključkom, druga je sa usb-c priključkom. 

Za MicroUSB verziju (NodeMCU ESP-32 sa 38 pinova) ne mogu pronaći pinout koji odgovara pločici, ali pinout za usb-c verziju odgovara stanju na pločici. Pinout za usb-c verziju: https://mischianti.org/esp32-nodemcu-32s-esp-32s-kit-high-resolution-pinout-datasheet-and-specs/. Slika sa tog sajta je dio ovog repoa.

## Task

Potrebno je simulirati sistem za detekciju anomalija faznog napona ili faznog struje na dalekovodu elektroenergetske mreže koristeći četiri ESP32 mikrokontrolera i Bluetooth Low Energy (BLE) protokol. Sistem za detekciju anomalija se sastoji od tri mikrokontrolera koji imaju ulogu BLE servera (periferni uređaji) i jednog kontrolera koji ima ulogu BLE klijenta (centralnog uređaja). Periferni uređaj (BLE server) obavlja četiri zadatka. Prvi zadatak je periodična sinhronizacija internog časovnika (clock-a) servera i klijenta, gdje se svakih 60 sekundi šalje informacija o trenutnoj vrijednosti internog časovnika servera koju klijent skladišti i koristi za sinhronizaciju časovnika. Drugi zadatak je odmjeravanje faznih vrijednosti napona i struje sa frekvencijom odmjeravanja od 1kHz, njihovo skladištenje i periodično slanje kontrolnih odmjeraka jednog perioda naizmjeničnog napona i struje (vremenski period od 20 ms) radi kontrolnog računanja trofazne snage. Odmjerci se skladište kao 16 bitni podatak, zbog praktičnog ograničenja analogno-digitalnog konvertora iskorištenog u ESP32 mikrokontroleru koji ima rezoluciju od 12 bita. Treći zadatak je detekcija anomalija prilikom odmjeravanja vrijednosti faznog napona i struje. Ukoliko vrijednost napona ili struje prelazi zadanu graničnu vrijednost (unaprijed ili dinamički definisanu), server šalje klijentu informaciju o detektovanju anomalije tj. trenutak u kojem se anomalija detektovala. Posljednji zadatak koji BLE server obavlja je slanje odmjeraka na zahtjev klijenta u trajanju od 1s prije i poslije trenutka koji je poslan kao dio zahtjeva. Ovaj zadatak se koristi za računanje snage  prije i poslije detekcije anomalije i analizu sistema. Centralni uređaj (BLE klijent) je zadužen za uspostavljanje veze i registraciju svih servisa i karakteristika sva tri periferna uređaja. Osim za uspostavljanje veze, zadužen je za sinhronizaciju internih časovnika svih uređaja, propagaciju informacije o detekciji anomalije, vremensko poravnavanje odmjeraka sve tri periferije za periodičnu analizu (izračun snage za jedan period) te vremensko poravnavanje i analizu odmjeraka oko trenutka detekcije anomalije.
Pored implementacije ovog načina prenosa podataka, potrebno je uraditi analizu i diskusiju rješenja koje bi neprekidno slalo odmjerke sa periferije na centralni uređaj.

## Steps

Predlažem postepen razvoj, tako da se prvo krene sa jednostavnim zadacima pa da se postepeno povećava kompleksnost. Neka izmišljena lista zadataka bi mogla ovako izgledati: 

1. Upali ledicu i ugasi ledicu sa nekim tajmerom (generalno ispis na serijski port, rad sa jednim uredjajem).
2. Razmjena podataka između dva uređaja i njihov ispis na serijski port.
3. Razmjena podataka između dva uređaja i uslovna akcija (paljenje ledice)
4. Komunikacija dva servera i jednog klijenta. Ispis poruka na serijski port.
5. Komunikacija dva servera i jednog klijenta i uslovna akcija. Asinhrona komunikacija.
6. Sinhronizacija dva uređaja (server i klijent) na osnovu razmjenjenih poruka. TODO: Osmisliti metodologiju kako da se poravnaju prema glavnom uređaju. Imaju interni clock koji otkliže, ali imamo i problem trajanja prenosa poruke. Prvo zanemariti vrijeme prenosa pa onda i njega uključiti. Jedna ideja za klijentsku stranu je slanje trenutnog vrijemena uređaja i vrijeme od prethodne sinhronizacije. Za serversku stranu potrebno je poslati trenutno vrijeme uređaja, vrijeme od prethodne sinhronizacije (razlika ovih vremena od prethodne sinhronizacije nam moze dati vrijeme propagacije signala) i razliku trenutnog sata servera i primljenog sata klijenta (takođe vrijeme propagacije, ali i mogućnost sinhronizacije da ova razlika uvijek bude ista). Ono što je potrebno uraditi je otkriti i najveći interval u kojem ovi satovi ne otkližu, te da to bude interval sinhronizacije. 
7. Uslovna akcija prenosa poruke. Na primjer, logička jedinica na jednom pinu uslovljava razmjenu komunikacije između uređaja (interrupt rutina). Dio ove poruke treba biti i vremenski potpis uređaja koji šalje poruku u obliku: trenutno vrijeme, vrijeme od prethodne sinhronizacije (TBD šta to tačno znači), vrijeme do naredne sinhronizacije (možda i ovo da bi se moglo interpolirati koliko dugo je potrebno
8. Neprekidno samplovanje signala na jednom uređaju u tri bafera. Da bi se omogućio prenos samplovanih podataka do trenutka $T_{1}$ trajanja $\Delta T$ i od trenutka $T_{1}$ istog trajanja $\Delta T$, potrebno je imati tri ciklična bafera u koje staje $2 * \Delta T$ podataka. Mislim da bi moglo i u bafer od $\Delta T$ podataka, ali pošto imamo vrijeme propagacije i obrade primljenih poruka, bolje za sada $2 * \Delta T$. Nisam siguran da može samplovati dok je unutar ISR-a, ali i to je nešto što se može probati namjestiti (trenutno nije neophodno.
9. Na uslovnu akciju, prenijeti stanje bafera za vrijeme $\Delta T$ prije akcije i tačan trenutak $T_{1}$ u kojem se interrupt dogodio. Ovo bi se trebalo dešavati na serverskoj strani, dok bi klijentska trebala da na osnovu trenutka $T_{1}$ izvuče svojih $2 * \Delta T$ trenutaka iz bafera i ispiše vrijednosti na serijski port (pošalje ih, nešto uradi samo da ima poravnat početak).
10. U priču dodati još jedan uređaj (kasnije još dva) koji bi imali ulogu servera. Proširiti tačku 9 sa emitovanjem broadcast signala za sve serverske uređaje, da pošalju klijentu svoje bafere. Ovdje dolazimo u problem šta vrijeme $T_{1}$ znači tom drugom (i trećem) uređaju. Sada imamo četiri clock-a, prvi clock je od klijenta, koji ima ulogu mastera i koji započinje proces sinhronizacije svojih slave uređaja (mislim da u BLE nema koncepta master-slave). Dakle, $CLK_{MASTER}$ je referentni clock i prema njemu se sinhronizuju $CLK_{SERVER_1}$, $CLK_{SERVER_2}$ i $CLK_{SERVER_3}$ i $MASTER$ uređaj vodi tabelu/listu za svaki clock, kada je odradio sinhronizaciju (interval na početku može biti isti, kasnije se može probati i on dinamički odrediti) i kako da odradi konverziju sa vremena tog kloka/uređaja na svoje vrijeme (u suštini deltu neku, koju bi odredio u nekom inicijalnom handshake procesu sa zahtjevima za komunikaciju svako N sekundi). Ako uređaj $SERVER_1$ pošalje broadcast da se dogodio interrupt, on šalje podatak desio se interrupt i vrijeme $T_{1}$ koji razumije i čita samo $MASTER$ uređaj, ostali uređaji kada zaprime taj broadcast signal, znaju da sigurno prethodnih $2 * \Delta T$ odmjeraka ne smiju pregaziti i da moraju sačuvati narednih $\Delta T$ odmjeraka. $MASTER$ uređaj kada obradi broadcast signal, i prevede vrijeme $T_{1}$ prvo na svoje vrijeme $T_{MASTER}$, a zatim na vremena $T_{SERVER_2}$ i $T_{SERVER_3}$ (pod pretpostavkom da se interrupt desio na uređaju $SERVER_1$) i proslijedi zahtjev za baferom dužine $2 * \Delta T$ oko trenutka $T_{1}$. Po prijemu svih podataka, na $MASTER$ uređaju se vrši poravnavanje.


TODO:

1. Objasniti/osmisliti inicijalni handshake
2. Objasniti/osmisliti koje uloge uređaji zauzimaju
3. Objasniti/osmisliti najbolji metod za samplovanje u bafere (najmanji baferi mogući)

## Useful links

Ovdje je sekcija sa korisnim linkovima, odnosno upoznavanjem prvo sa pločicom a zatim sa BLE standardom.

O čipu: https://lastminuteengineers.com/getting-started-with-esp32/

Pinout: https://mischianti.org/esp32-nodemcu-32s-esp-32s-kit-high-resolution-pinout-datasheet-and-specs/

BLE protokol: 

IDE: https://www.arduino.cc/en/software

Instalacije IDE-a: https://randomnerdtutorials.com/installing-esp32-arduino-ide-2-0/


