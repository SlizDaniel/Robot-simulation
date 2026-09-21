# Robot Simulator

Prosty symulator ruchu robota mobilnego w środowisku 2D. Projekt modeluje robota i prostokątne przeszkody, oblicza ich obrócone obrysy, wykrywa kolizje algorytmem SAT (*Separating Axis Theorem*) oraz zatrzymuje robota przed przewidzianą kolizją lub opuszczeniem środowiska.

Aktualna wersja: **0.1**.

## Zakres v0.1

- ruch robota prostoliniowy i po łuku;
- prostokątny robot oraz prostokątne, obrócone przeszkody;
- obliczanie wierzchołków obróconych prostokątów;
- wykrywanie kolizji robot–przeszkoda przez SAT;
- sprawdzanie, czy cały obrys robota mieści się w środowisku;
- przewidywanie następnego kroku ruchu przed zmianą stanu robota;
- zatrzymanie robota przed przewidzianą kolizją lub wyjazdem poza środowisko;
- testy jednostkowe oparte na GoogleTest.

Projekt jest obecnie biblioteką C++ z testami, a nie gotową aplikacją konsolową lub graficzną.

## Wymagania

- kompilator C++ obsługujący C++17;
- CMake 3.16 lub nowszy;
- dostęp do Internetu podczas pierwszej konfiguracji testów, aby CMake mógł pobrać GoogleTest.

## Budowanie i uruchamianie testów

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Biblioteka jest budowana jako `robot_simulator`. Konfiguracja testowa tworzy także wykonywalny zestaw testów `robot_simulator_tests`.

W aktualnej wersji zestaw obejmuje 50 testów geometrii, robota, przeszkód, środowiska i symulacji.

## Model danych

Podstawowe typy są zdefiniowane w `include/robot_simulation/types.hpp`.

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
```

- `Pose` opisuje środek obiektu oraz jego orientację `theta` w radianach.
- `Velocity` zawiera prędkość liniową i kątową robota.
- `Rectangle` przechowuje szerokość oraz długość. Długość leży w lokalnym kierunku osi X obiektu, a szerokość w lokalnym kierunku osi Y.
- Projekt nie narzuca jednostek. Wszystkie wartości muszą jednak być podane w jednym, spójnym układzie jednostek.

## Układ współrzędnych

Środek `Environment` znajduje się w punkcie `(0, 0)`.

Dla środowiska o rozmiarze:

```cpp
Rectangle{width, length}
```

obowiązują granice:

```text
x: od -length / 2 do +length / 2
y: od -width / 2 do +width / 2
```

Pozycja robota i przeszkody oznacza środek ich prostokątnego obrysu.

## Główne klasy

### `Robot`

Przechowuje pozycję, prędkość i rozmiar. Udostępnia:

- `move(dt)` — wykonuje ruch prostoliniowy lub ruch po łuku;
- `stop()` — zeruje prędkość liniową i kątową;
- `getRobotCorners()` — zwraca cztery wierzchołki obróconego obrysu.

Konstruktor odrzuca zerową i ujemną szerokość lub długość przez `std::invalid_argument`.

### `Obstacle`

Przechowuje pozycję i rozmiar nieruchomej przeszkody. Metoda `getObstacleCorners()` zwraca jej cztery wierzchołki. Konstruktor waliduje dodatnie wymiary tak samo jak `Robot`.

### `Environment`

Przechowuje rozmiar świata i kolekcję przeszkód.

- `addObstacle()` dodaje przeszkodę do środowiska;
- `collides()` sprawdza kolizję robota z dowolną przeszkodą;
- `isRobotInside()` sprawdza, czy wszystkie rogi robota są w granicach środowiska.

Konstruktor odrzuca zerowy i ujemny rozmiar środowiska.

### `Simulation`

Łączy robota ze środowiskiem i wykonuje pojedynczy krok o ustalonym czasie `dt`.

`update()` nie przesuwa robota bezpośrednio. Najpierw tworzy kopię robota, wykonuje na niej przewidywany ruch i sprawdza przewidywany stan:

1. czy robot pozostanie w środowisku;
2. czy nie zderzy się z przeszkodą;
3. dopiero wtedy przesuwa prawdziwego robota.

Jeżeli przewidywany krok jest niedozwolony, robot zostaje zatrzymany na ostatniej bezpiecznej pozycji.

Wynik kroku ma postać:

```cpp
struct SimulationUpdate {
    StopReason stop_reason;
    bool step_accepted;
};
```

Możliwe powody zatrzymania:

```cpp
StopReason::None
StopReason::OutOfEnvironment
StopReason::ObstacleCollision
```

`step_accepted` oznacza, że krok został zaakceptowany przez symulację. Nie musi oznaczać fizycznej zmiany pozycji, np. gdy robot ma już zerową prędkość.

## Kolizje

Kolizje prostokątów są wykrywane algorytmem SAT.

1. Dla obu obróconych prostokątów wybierane są osie zgodne z ich bokami.
2. Wszystkie wierzchołki są rzutowane na każdą z osi.
3. Jeżeli na choć jednej osi przedziały rzutów są rozłączne, kolizji nie ma.
4. Jeżeli przedziały nakładają się na wszystkich osiach, występuje kolizja.

Dotknięcie przeszkody bokiem lub narożnikiem jest traktowane jako kolizja. Natomiast robot dotykający granicy środowiska nadal jest uznawany przez `isRobotInside()` za znajdującego się wewnątrz; kolejny krok, który wyprowadzi go poza granicę, zostanie odrzucony.

## Przykład użycia

```cpp
#include <robot_simulation/environment.hpp>
#include <robot_simulation/obstacle.hpp>
#include <robot_simulation/robot.hpp>
#include <robot_simulation/simulation.hpp>

using namespace robot_simulation;

Rectangle environment_size{20.0, 30.0};
Environment environment(environment_size);

Obstacle obstacle(
    Pose{4.5, 0.0, 0.0},
    Rectangle{2.0, 4.0}
);
environment.addObstacle(obstacle);

Robot robot(
    Pose{0.0, 0.0, 0.0},
    Velocity{2.0, 0.0},
    Rectangle{2.0, 4.0}
);

Simulation simulation(0.5, robot, environment);
const SimulationUpdate result = simulation.update();

if (!result.step_accepted) {
    // Odczytaj result.stop_reason i obsłuż zatrzymanie robota.
}
```

## Struktura projektu

```text
include/robot_simulation/  Publiczne nagłówki biblioteki
src/                       Implementacje klas i geometrii
tests/                     Testy GoogleTest
CMakeLists.txt             Konfiguracja budowania CMake
```

## Ograniczenia v0.1

- Kolizja jest sprawdzana dyskretnie, tylko dla stanu po jednym kroku symulacji. Przy dużym `dt` lub dużej prędkości robot może przeskoczyć przez bardzo cienką przeszkodę.
- Przy przewidzianej kolizji robot zatrzymuje się na pozycji sprzed kroku; symulator nie wyznacza dokładnego punktu kontaktu.
- Biblioteka nie zawiera jeszcze aplikacji demonstracyjnej, interfejsu graficznego, logowania ani zapisu sceny.
- Walidowane są dodatnie wymiary i dodatni krok czasowy. Wersja 0.1 nie odrzuca jeszcze wartości `NaN` oraz nieskończoności.

## Możliwe kierunki rozwoju

- ciągłe wykrywanie kolizji lub adaptacyjny krok czasowy;
- dokładne wyznaczanie punktu kontaktu;
- walidacja `NaN` i nieskończoności;
- sterowanie prędkością robota po jego utworzeniu;
- aplikacja konsolowa lub GUI;
- wizualizacja środowiska i trajektorii;
- automatyczne uruchamianie testów w CI.

## Licencja

Projekt nie ma jeszcze określonej licencji. Przed publicznym udostępnieniem lub użyciem przez inne osoby warto dodać plik `LICENSE`.
