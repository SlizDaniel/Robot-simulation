# Frontend symulatora

Pliki `index.html`, `styles.css` i `app.js` są serwowane bez procesu budowania przez istniejący serwer C++. Uruchom serwer z katalogu głównego repozytorium i otwórz http://127.0.0.1:8080/.

## Edycja i widok

- Formularze i przeciąganie zmieniają projekt; Initialize wysyła SceneConfig do `/api/simulation/init`.
- Import i eksport JSON, duplikowanie przeszkód, cofanie do 50 edycji oraz odrzucenie zmian działają lokalnie. Import nie inicjalizuje sesji automatycznie.
- Kliknięcie obiektu na planszy wybiera jego formularz. Kółko zmienia zoom, przeciągnięcie pustego tła przesuwa widok, a Dopasuj przywraca całą scenę.
- Warstwy siatki, czujników i trajektorii można wyłączać. Płynny ruch jest wyłącznie interpolacją wizualną; stan i czas pochodzą z backendu. Preferencja ograniczenia animacji jest respektowana.
- Czujniki pokazują rzeczywiste pole widzenia i zakres min/max. Ich zbiorcza odległość znajduje się w telemetrii; API nie pozwala ustalić punktu trafienia.
- Trajektoria zawiera maksymalnie 5000 ostatnich pozycji. Synchronizacja rozpoczyna ją od odczytanej pozycji, ponieważ API nie zwraca historii.
- `j.d.` oznacza jednostkę długości, a `j.t.` jednostkę czasu. Wszystkie kąty są w radianach.

## Połączenie i sterowanie

Start wysyła kroki sekwencyjnie. Pause zatrzymuje kolejne żądania, ale już wysłany krok może się jeszcze zakończyć. Żądania mają limit 8 sekund. Po niepotwierdzonej operacji sterowanie jest blokowane do synchronizacji; frontend nigdy nie ponawia automatycznie mutujących żądań. Przycisk Połącz / synchronizuj odczytuje konfigurację i stan, zachowując lokalne edycje.

Backend ma jedną wspólną sesję. Synchronizacja pomaga po zmianach z innej karty, lecz osobne `/config` i `/state` nie zapewniają atomowego odczytu przy równoczesnej inicjalizacji przez innego klienta. Pełne rozwiązanie wymagałoby wersjonowania sesji w API.

## Testy interfejsu

Tylko do uruchomienia testów potrzebny jest Node.js i Playwright:

```bash
cd frontend
npm ci
npx playwright install chromium
npm test
npm run test:e2e
```

`npm test` używa Chromium i deterministycznych odpowiedzi API, aby sprawdzić również timeout i błędy połączenia. `npm run test:e2e` uruchamia własny proces `build/robot_simulator_server` i sprawdza UI przez prawdziwe endpointy. Przed tym testem zbuduj serwer C++ i zwolnij port `127.0.0.1:8080`; test nie podłącza się do cudzego procesu. Proces jest zamykany po zakończeniu testów.
