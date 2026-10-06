# Robot Simulator

Prosty symulator ruchu prostokątnego robota mobilnego w środowisku 2D, napisany w C++17. Projekt modeluje ruch prostoliniowy i po łuku, prostokątne przeszkody, granice środowiska oraz kolizje wykrywane algorytmem SAT (*Separating Axis Theorem*).

Aktualna wersja: **0.3**.

Projekt zawiera bibliotekę C++, aplikację demonstracyjną działającą w terminalu oraz testy jednostkowe GoogleTest.

## Najważniejsze możliwości

- ruch robota prostoliniowy i po łuku;
- zmiana prędkości liniowej i kątowej podczas działania;
- obliczanie wierzchołków obróconego robota i przeszkód;
- wykrywanie kolizji obróconych prostokątów algorytmem SAT;
- sprawdzanie, czy cały obrys robota pozostaje w środowisku;
- przewidywanie następnej pozycji bez modyfikowania stanu robota;
- odrzucanie kroku prowadzącego do kolizji lub opuszczenia środowiska;
- wykonywanie pojedynczych kroków lub serii kroków;
- zliczanie zaakceptowanych kroków i czasu symulacji;
- zapisywanie historii zaakceptowanej trajektorii;
- symulowane czujniki odległości wykorzystujące ray-casting dla przeszkód i granic środowiska;
- pomiary czujników zwracane przez każdy krok symulacji;
- aplikacja konsolowa prezentująca przebieg symulacji i pomiary czujnika;
- 146 testów jednostkowych.

## Co nowego w v0.3

W porównaniu z v0.2 dodano:

- `DistanceSensor` z pozycją i orientacją względną wobec robota;
- ray-casting granic środowiska oraz prostokątnych przeszkód;
- minimalny i maksymalny zasięg oraz obsługę obiektów poza zakresem;
- wybór najbliższego echa spośród przeszkód i granic środowiska;
- wyniki ray-castingu dla każdego czujnika w `SimulationUpdate`;
- konfigurowalną, dodatnią i nieparzystą liczbę promieni w `Simulation`;
- prezentację pomiarów po każdym kroku w aplikacji demonstracyjnej;
- testy geometrii, czujnika i integracji czujników z symulacją.

## Wymagania

- kompilator C++ obsługujący C++17;
- CMake 3.16 lub nowszy;
- dostęp do Internetu podczas pierwszej konfiguracji z testami, ponieważ CMake pobiera GoogleTest 1.14.0.

## Budowanie projektu

### Aplikacja i testy

W katalogu głównym projektu wykonaj:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
```

Znaczenie katalogów:

- `-S .` wskazuje bieżący katalog jako katalog źródeł;
- `-B build` umieszcza wygenerowane pliki i wyniki kompilacji w katalogu `build/`.

### Budowanie bez testów

Jeżeli chcesz zbudować tylko bibliotekę i aplikację, bez pobierania GoogleTest:

```bash
cmake -S . -B build -DBUILD_TESTING=OFF
cmake --build build
```

### Dostępne targety

| Target | Rodzaj | Przeznaczenie |
| --- | --- | --- |
| `robot_simulator` | biblioteka | model robota, środowiska, przeszkód, geometrii i symulacji |
| `robot_simulator_app` | program | terminalowa demonstracja symulatora |
| `robot_simulator_tests` | program testowy | testy GoogleTest, dostępne przy `BUILD_TESTING=ON` |

Wybrany target można zbudować osobno:

```bash
cmake --build build --target robot_simulator_app
```

## Uruchamianie aplikacji

Po zbudowaniu targetu uruchom:

```bash
./build/robot_simulator_app
```

Aplikacja tworzy środowisko o długości `30` i szerokości `12`, robota rozpoczynającego ruch w punkcie `(-8, 0)` oraz przeszkodę w punkcie `(8, 0)`.

Demonstracja składa się z dwóch faz:

1. robot wykonuje cztery kroki z prędkością liniową `2.0` i kątową `0.25 rad/s`;
2. prędkość zostaje zmieniona na liniową `4.0` i kątową `0.15 rad/s`, po czym symulacja próbuje wykonać maksymalnie dziesięć kroków.

Robot ma jeden czujnik odległości z trzema promieniami. Po każdej próbie kroku program wypisuje numer próby, numer zaakceptowanego kroku, czas, stan kroku oraz wynik ray-castingu z odległością najbliższego echa. W aktualnym scenariuszu zakrzywiona trajektoria kończy się zatrzymaniem robota przed opuszczeniem środowiska.

## Uruchamianie testów

Po skonfigurowaniu projektu z `BUILD_TESTING=ON` wykonaj:

```bash
ctest --test-dir build --output-on-failure
```

Projekt zawiera 146 testów:

| Obszar | Liczba testów |
| --- | ---: |
| geometria i ray-casting | 37 |
| robot | 19 |
| przeszkody | 7 |
| środowisko i kolizje | 20 |
| czujnik odległości | 45 |
| symulacja | 18 |
| **razem** | **146** |

Testy obejmują między innymi walidację wymiarów i kroku czasowego, ruch prostoliniowy i kołowy, obrócone prostokąty, SAT, granice środowiska, ray-casting przeszkód i granic, zasięgi czujnika, integrację czujników z symulacją, historię trajektorii oraz liczniki czasu i kroków.

## Model danych

Podstawowe typy znajdują się w `include/robot_simulation/types.hpp`:

```cpp
struct Pose {
    double x;
    double y;
    double theta;
};

struct Velocity {
    double linearVelocity;
    double angularVelocity;
};

struct Rectangle {
    double width;
    double length;
};

struct Point {
    double x;
    double y;
};

using RectangleCorners = std::array<Point, 4>;
```

- `Pose` opisuje środek obiektu i jego orientację `theta` w radianach;
- `Velocity` przechowuje prędkość liniową oraz kątową w radianach na jednostkę czasu;
- `Rectangle` przechowuje szerokość i długość obiektu;
- `Point` reprezentuje punkt w układzie 2D;
- `RectangleCorners` zawiera cztery wierzchołki prostokąta.

Projekt nie narzuca jednostki długości ani czasu. Wszystkie dane wejściowe muszą jednak używać jednego, spójnego układu jednostek.

## Układ współrzędnych

Środek `Environment` znajduje się w punkcie `(0, 0)`. Dla rozmiaru:

```cpp
Rectangle{width, length}
```

granice środowiska wynoszą:

```text
x: od -length / 2 do +length / 2
y: od -width  / 2 do +width  / 2
```

Pozycja robota i przeszkody oznacza środek ich prostokątnego obrysu. Długość leży w lokalnym kierunku osi X obiektu, a szerokość w lokalnym kierunku osi Y.

## Główne elementy biblioteki

### `Robot`

`Robot` przechowuje pozycję, prędkość, rozmiar i historię trajektorii.

Najważniejsze metody:

- `move(dt)` wykonuje jeden ruch i dodaje nową pozycję do trajektorii;
- `robotNextPose(dt)` oblicza przewidywaną pozycję bez zmiany robota i trajektorii;
- `setVelocity(linear, angular)` ustawia obie składowe prędkości;
- `stop()` zeruje prędkość liniową i kątową;
- `getRobotCorners()` zwraca wierzchołki aktualnego obrysu;
- `getTrajectory()` udostępnia historię pozycji jako stałą referencję.

Trajektoria zawsze rozpoczyna się od pozycji przekazanej do konstruktora. Każde wywołanie `move()` dodaje dokładnie jeden punkt.

Konstruktor odrzuca zerową i ujemną szerokość lub długość przez `std::invalid_argument`.

### `Obstacle`

`Obstacle` reprezentuje nieruchomą, prostokątną przeszkodę. Przechowuje jej pozycję i rozmiar, a `getObstacleCorners()` zwraca cztery wierzchołki obróconego obrysu.

Konstruktor odrzuca zerową i ujemną szerokość lub długość.

### `Environment`

`Environment` przechowuje rozmiar świata oraz kolekcję przeszkód.

- `addObstacle()` dodaje kopię przeszkody do środowiska;
- `collides()` sprawdza kolizję robota lub podanego przewidywanego obrysu z przeszkodami;
- `isRobotInside()` sprawdza, czy wszystkie narożniki robota lub obrysu mieszczą się w środowisku;
- `getObstacles()` zwraca stałą referencję do kolekcji przeszkód.

Dotknięcie granicy środowiska jest dozwolone. Dopiero narożnik znajdujący się poza granicą powoduje odrzucenie pozycji.

### `DistanceSensor`

`DistanceSensor` jest montowany względem układu robota przez `Pose` przekazane do konstruktora. Metoda `getWorldPose()` wyznacza jego aktualną pozycję i orientację w świecie.

Najważniejsza metoda pomiarowa to:

```cpp
RaycastResult nearestObjectDetected(
    const Environment& environment,
    int ray_count,
    const Pose& robot_pose
) const;
```

`ray_count` musi być dodatni i nieparzysty; jeden promień zawsze biegnie w centralnym kierunku czujnika. Pomiar uwzględnia przeszkody oraz granice środowiska i wybiera najbliższe echo. `distance_to_object` zachowuje fizyczną odległość także wtedy, gdy echo jest poza zakresem czujnika.

```cpp
struct RaycastResult {
    bool object_detected;
    double distance_to_object;
    DetectionType detection_type;
};
```

- `OBJECTDETECTED` oznacza echo w przedziale `[min_distance, max_distance]`;
- `OBJECTOUTOFRANGE` oznacza echo poza tym przedziałem;
- `NOOBJECTDETECTED` oznacza brak echa; dla `nearestObjectDetected()` czujnik znajdujący się wewnątrz poprawnego prostokątnego środowiska zwykle zawsze otrzyma echo granicy.

Robot może przechowywać wiele czujników przez `addDistanceSensor()`, a `getDistanceSensors()` udostępnia je jako stałą referencję.

### `Simulation`

`Simulation` łączy istniejący obiekt `Robot` z istniejącym obiektem `Environment` i przechowuje do nich referencje. Robot i środowisko muszą więc istnieć przez cały czas życia symulacji.

Konstruktor przyjmuje dodatni czas jednego kroku `dt` oraz opcjonalną liczbę promieni używanych przez czujniki. Liczba promieni musi być dodatnia i nieparzysta; domyślna wartość to `1`.

`update()` wykonuje pojedynczy krok:

1. oblicza przewidywaną pozycję robota;
2. wyznacza przewidywane narożniki obrysu;
3. sprawdza granice środowiska;
4. sprawdza kolizję z przeszkodami;
5. akceptuje ruch dopiero wtedy, gdy przewidywany stan jest bezpieczny.
6. wykonuje ray-casting dla każdego czujnika robota i dołącza wyniki do `SimulationUpdate`.

Jeżeli krok jest niedozwolony, robot pozostaje w ostatniej bezpiecznej pozycji, jego prędkość zostaje wyzerowana, a odrzucony punkt nie trafia do trajektorii i nie zwiększa liczników symulacji.

`runSteps(step_count)` wielokrotnie wywołuje `update()` aż do wykonania zadanej liczby kroków albo do pierwszego odrzuconego kroku. Argument oznacza liczbę kroków, a nie czas w sekundach.

`getSimulationStep()` zwraca całkowitą liczbę zaakceptowanych kroków od utworzenia symulacji. `getSimulationTime()` zwraca tę liczbę pomnożoną przez `dt`.

## Wyniki symulacji

Pojedynczy krok zwraca:

```cpp
struct SimulationUpdate {
    StopReason stop_reason;
    bool step_accepted;
    std::vector<RaycastResult> raycast_results;
};
```

Seria kroków zwraca:

```cpp
struct SimulationResult {
    std::size_t executed_steps;
    bool simulation_completed;
    StopReason stop_reason;
};
```

- `executed_steps` dotyczy tylko bieżącego wywołania `runSteps()`;
- `simulation_completed` oznacza, że wykonano wszystkie żądane kroki;
- `stop_reason` wyjaśnia przyczynę wcześniejszego zatrzymania.
- `raycast_results` zawiera jeden pomiar dla każdego czujnika robota, także gdy próba kroku została odrzucona.

Możliwe przyczyny zatrzymania:

```cpp
StopReason::None
StopReason::OutOfEnvironment
StopReason::ObstacleCollision
```

Zaakceptowany krok nie musi zmieniać pozycji, na przykład gdy robot ma zerową prędkość.

## Wykrywanie kolizji

Kolizje pomiędzy robotem i przeszkodami są wykrywane za pomocą SAT:

1. wybierane są osie zgodne z bokami robota i przeszkody;
2. narożniki obu prostokątów są rzutowane na każdą oś;
3. rozłączne przedziały na dowolnej osi oznaczają brak kolizji;
4. nakładanie się przedziałów na wszystkich osiach oznacza kolizję.

Dotknięcie przeszkody bokiem lub narożnikiem jest traktowane jako kolizja.

## Przykład użycia biblioteki

```cpp
#include <robot_simulation/environment.hpp>
#include <robot_simulation/distance_sensor.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/simulation.hpp>

using namespace robot_simulation;

Rectangle environment_size{20.0, 30.0};
Environment environment(environment_size);

environment.addObstacle(Obstacle(
    Pose{6.5, 0.0, 0.0},
    Rectangle{2.0, 4.0}
));

Robot robot(
    Pose{0.0, 0.0, 0.0},
    Velocity{2.0, 0.0},
    Rectangle{2.0, 4.0}
);

robot.addDistanceSensor(DistanceSensor(0.0, 10.0, Pose{}, pi / 2.0));
Simulation simulation(0.5, robot, environment, 3);
const SimulationUpdate update = simulation.update();

if (!update.raycast_results.empty()) {
    const RaycastResult& measurement = update.raycast_results.front();
    // measurement opisuje najbliższe echo pierwszego czujnika.
}

const auto& trajectory = robot.getTrajectory();
```

## Struktura projektu

```text
robot_simulator/
├── app/
│   └── main.cpp                 Aplikacja demonstracyjna
├── include/robot_simulation/    Publiczne nagłówki biblioteki
├── src/                         Implementacje biblioteki
├── tests/                       Testy GoogleTest i funkcje pomocnicze
├── CMakeLists.txt               Konfiguracja targetów CMake
├── README.md                    Dokumentacja projektu
└── .gitignore                   Reguły ignorowania plików lokalnych
```

## Ograniczenia v0.3

- Kolizja jest sprawdzana dyskretnie dla stanu na końcu kroku. Przy dużym `dt`, dużej prędkości albo cienkiej przeszkodzie robot może przeskoczyć przez przeszkodę pomiędzy kolejnymi stanami.
- Przy przewidzianej kolizji robot pozostaje w pozycji sprzed kroku. Symulator nie wyznacza dokładnego czasu ani punktu kontaktu.
- Walidowane są dodatnie wymiary i dodatni krok czasowy, ale wartości `NaN` oraz nieskończoności nie są jeszcze odrzucane.
- Historia trajektorii rośnie z każdym wykonanym ruchem i nie ma obecnie limitu rozmiaru.
- Czujnik nie rozróżnia w `RaycastResult`, czy echo pochodzi od przeszkody czy granicy środowiska.
- Model pomiaru nie uwzględnia szumu ani błędów systematycznych.
- Aplikacja demonstracyjna nie przyjmuje jeszcze parametrów z wiersza poleceń ani pliku konfiguracyjnego.
- Projekt nie zawiera jeszcze interfejsu graficznego, zapisu sceny ani automatycznego CI.

## Możliwe kierunki rozwoju

- ciągłe wykrywanie kolizji lub adaptacyjny krok czasowy;
- dokładne wyznaczanie czasu i punktu kontaktu;
- walidacja `NaN` i nieskończoności;
- opcjonalny limit lub wyłączanie historii trajektorii;
- konfiguracja sceny z argumentów programu lub pliku;
- wizualizacja środowiska i trajektorii;
- zapis i odczyt sceny;
- automatyczne budowanie i testowanie w CI.

## Licencja

Projekt nie ma jeszcze określonej licencji. Przed publicznym udostępnieniem do ponownego użycia należy dodać plik `LICENSE`.
