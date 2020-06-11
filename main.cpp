#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <random>
#include <chrono>
#include <vector>
#include <map>

#include "ncurses.h"


// Zasoby
const unsigned int STATIONS_NUMBER = 4;
const unsigned int RAILS_NUMBER = STATIONS_NUMBER;
const unsigned int CARRIAGES_NUMBER = 5;               // Liczba wagonow na jedna stacje
const unsigned int PATHS_NUMBER = RAILS_NUMBER + 2;    // liczba sciezek = wzdluz torow i po krzyzu

// Watki
const unsigned int TRAINS_NUMBER = 10;
const unsigned int PASSENGERS_NUMBER = 13;



// Klasa dworzec. Na dworcu moze przebywac kilka pociagow.
class Station
{
    public:
        int id;
        std::shared_timed_mutex smtx;       // shared mutex
        std::mutex mtx;

        std::atomic<bool> train_on_station{false};
        std::atomic<int> train_id;
        std::atomic<int> train_ride_time; 
};


// Klasa wagony kolejowe - informuje o stanie wagonow na danej stacji oraz pozwala na podlaczanie wagonow do skladu
class Carriages
{
    public:
        int station_id;
        std::mutex mtx;
        std::atomic<int> free_carriages;
        std::condition_variable cond;
};


// Klasa tory. Na torach moze znajdowac sie w tym samym czasie tylko jeden pociag.
class Rail
{
    public:
        int id;
        std::mutex mtx;
};


// Sciezki pomiedzy stacjami. Po sciezkach poruszaja sie pasazerowie, ktorzy nie jada pociagiem
class Path
{
    public:
        int id;
        std::shared_timed_mutex smtx;
        std::mutex mtx;
        std::atomic<int> passengers_number;
        std::condition_variable cond;
};


// Klasa mapa kolejowa - zbudowana z dworcow polaczonych torami
class RailwayMap
{
    public:
        std::array<Station, STATIONS_NUMBER> stations;
        std::array<Carriages, STATIONS_NUMBER> carriages;
        std::array<Rail, RAILS_NUMBER> rails;
        std::array<Path, PATHS_NUMBER> paths;

        std::atomic<bool> ready{false};                 // zmienna atomowa informujaca o stanie mapy
        std::mt19937 rng{std::random_device{}()};

        RailwayMap();
        
    // Przypisane stacjom, torom i wagonom id, takze przypisanie liczby wagonow na kazdej stacji
    void setStationsAndRails()
    {
        for (int i=0; i<STATIONS_NUMBER; i++)
        {
            stations[i].id = i;
            rails[i].id = i;
            carriages[i].station_id = i; 
            carriages[i].free_carriages.store(CARRIAGES_NUMBER, std::memory_order_seq_cst);      // liczba wolnych wagonow na stacji
        }
    }

    // Przypisanie sciezka id
    void setPaths()
    {
        for (int i=0; i<PATHS_NUMBER; i++)
        {
            paths[i].id = i;
        }
        // path 0: S0 <-> S1
        // path 1: S1 <-> S2
        // path 2: S2 <-> S3
        // path 3: S3 <-> S4
        // path 4: S0 <-> S2
        // path 5: S1 <-> S3

    }
};

// Konstruktor przypisujacy id zasobom na mapie
RailwayMap::RailwayMap()
{
    setStationsAndRails();
    setPaths();
}



// Klasa pociag
class Train
{
    public:
        int id;             // ID pociagu
        int c_station;      // ID obecnej stacji
        int c_railF;        // ID obecnego toru

        std::thread th;                     // Watek pociagu
        RailwayMap &map;                    // Referencja do mapy i zasobow

        std::mt19937 rng{std::random_device{}()};

        bool end = false;
        int state = -1;     // -1 - waiting, 0 - stop, 1 - ride
        int progress = 0;   // postep w przejazdzie pomiedzy stacjami

        Train(int _id, RailwayMap &_map, int _station,  int _railF)
        : id(_id), map(_map), c_station(_station), c_railF(_railF), th(&Train::life, this) {}       // Konstruktor pociagu
        

    // Cykl zycia pociagu: postuj -> jazda
    void life() 
    {
        while(!map.ready)
        {
            std::this_thread::yield();      // yield() - umozliwia podjecie proby wywlaszczenia biezacego watku
                                            //           i przydzielenia czasu innemu watkowi
        }

        // Gdy mapa jest gotowa i !end, cykl pociagu: postoj -> jazda
        while (!end)
        {
            stop();
            ride();
        }
        return;
    }


    // Stop pociagu na stacji
    void stop()
    {
        std::shared_lock<std::shared_timed_mutex> station_lock(map.stations[c_station].smtx);       // zablokowanie dzielonego watku - stacji

        state = 0;  // 0 - stop
        int time = std::uniform_int_distribution<int>(40, 60)(rng);

        for (auto i=1; i<=time; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));        // wstrzymanie wykonywania biezacego watku na 0.1 s

            if (end)
            {
                return;
            }
        }
    }


    // Przemieszczanie sie pociagu wraz z wagonami miedzy stacjami
    void ride()
    {
        // Uzycie zmiennej warunkowej: blokowanie innych pociagow tak dlugo, az na stacji bedzie wystarczajaca liczba wagonow

        std::unique_lock<std::mutex> carriages_lock(map.carriages[c_station].mtx);     // Zablokowanie watku - stanu wagonow na danej stacji
        
        while (map.carriages[c_station].free_carriages.load(std::memory_order_seq_cst) < 1 && !end)   // Poki na stacji nie ma wagonow
        {
            map.carriages[c_station].cond.wait(carriages_lock);                 // czekaj (wait - atomowe zwolnienie mutexa i uspienie watku)
        }
        if (end)
        {
            return;
        }

        map.carriages[c_station].free_carriages.fetch_sub(1, std::memory_order_seq_cst);      // Gdy wagon wolny, wez jeden
        
        std::lock_guard<std::mutex> rail_lock(map.rails[c_railF].mtx);          // zablokowanie watku - torow        
        
        int time = std::uniform_int_distribution<int>(40, 60)(rng);

        ////////////////////////////
        // {
        //     std::lock_guard<std::mutex> passengers_lock(map.stations[c_station].mtx);
        //     map.stations[c_station].train_on_station = true;                            // zmiana zmiennej atomowej - pociag gotowy do przyjecia pasazerow na danej stacji
        //     map.stations[c_station].train_id.exchange(id, std::memory_order_seq_cst);   // zapisz id pociagu, ktory jest gotowy do przyjecia pasazerow
        //     map.stations[c_station].train_ride_time.exchange(time, std::memory_order_seq_cst);  // zapisz czas podrozy pociagu
        // }
        /////////////////////////////

        state = 1;  // 1 - ride
        

        for (auto i=0; i<=time; i++)
        {
            double p = (double) i / (double) time;
            progress = std::round(p * 100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));        // wstrzymanie wykonywania biezacego watku na 0.1 s

            if (end)
            {
                return;
            }
        }

        // Zmiana danych pociagu, po przemieszczeniu sie
        if (map.stations[c_station].id == STATIONS_NUMBER-1)    // jesli jedzie z ostatniej stacji do pierwszej
        {
            c_station = 0;
            c_railF = 0;
            map.carriages[c_station].free_carriages.fetch_add(1, std::memory_order_seq_cst);
        }
        else
        {
            c_station += 1;
            c_railF += 1;
            map.carriages[c_station].free_carriages.fetch_add(1, std::memory_order_seq_cst);
        }
        carriages_lock.unlock();
        map.carriages[c_station].cond.notify_one();     // Odblokuj jeden watek czekajacy na wagony

    }
};


// Klasa pasazer. Pasazerowie moga podrozowac pociagami lub poruszac sie miedzy stacjami na piechote po sciezkach.
class Passenger
{
    public:
        int id;
        int c_train;        // ID pociagu, ktorym jedzie pasazer
        int c_path;         // nr sciezki, ktora sie porusza
        int c_station;      // ID stacji, na ktorej przebywa pasazer

        //bool isTrainOnStation = false;
        std::condition_variable cond;

        std::thread th;     // Watek pasazera
        RailwayMap &map;    // Referencja do mapy i zasobow

        std::mt19937 rng{std::random_device{}()};

        bool end = false;
        int state = -1;     // -1 - not ready, 0 - wait, 1 - ride, 2 - walk
        int progress = 0;   // postep w przejazdzie pomiedzy stacjami/przejsciu na piechote miedzy stacjami

        Passenger(int _id, RailwayMap &_map, int _station)
        : id(_id), map(_map), c_station(_station), th(&Passenger::life, this) {}


    // Cykl zycia pasazera: czekaj na pociag -> jedz pociagiem -> idz na inna stacje
    void life() 
    {
        while(!map.ready)
        {
            std::this_thread::yield();      // yield() - umozliwia podjecie proby wywlaszczenia biezacego watku
                                            //           i przydzielenia czasu innemu watkowi
        }

        // Gdy mapa jest gotowa i !end, cykl pasazera: czekaj -> jazda pociagiem -> spacer
        while (!end)
        {
            wait();
            //ride();
            walk();
        }
        return;
    }


    // Pasazer czeka na stacji
    void wait()
    {
        std::shared_lock<std::shared_timed_mutex> station_lock(map.stations[c_station].smtx);       // zablokowanie dzielonego watku - stacji

        state = 0;  // 0 - wait
        int time = std::uniform_int_distribution<int>(30, 50)(rng);

        for (auto i=1; i<=time; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));        // wstrzymanie wykonywania biezacego watku na 0.1 s

            if (end)
            {
                return;
            }
        }
    }


    // Pasazer jedzie pociagiem
    void ride()
    {
        std::unique_lock<std::mutex> station_lock(map.stations[c_station].mtx);     // zablokuj mutex i znajdz pociag
        
        while (!map.stations[c_station].train_on_station && !end)       // jesli na stacji nie ma pociagu
        {  
            cond.wait(station_lock);                                    // czekaj
        }
        if (end)
        {
            return;
        }

        c_train = map.stations[c_station].train_on_station.load(std::memory_order_seq_cst);     // jesli pociag jest - wsiadz i zapisz id pociagu
        int time = map.stations[c_station].train_ride_time.load(std::memory_order_seq_cst);     // a takze pobierz informacje o czasie podrozy
        station_lock.unlock();    // odblokuj mutex dla innego pasazera, ktory czeka na pociag
        cond.notify_one();                                    

        state = 1;      // 1 - ride

        for (auto i=1; i<=time; i++)
        {
            double p = (double) i / (double) time;
            progress = std::round(p * 100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));        // wstrzymanie wykonywania biezacego watku na 0.1 s
            if (end)
            {
                return;
            }
        }

        // Zaaktualizuj stacje na ktorej obecnie przebywa pasazer
        if (map.stations[c_station].id == STATIONS_NUMBER-1)    // jesli jedzie z ostatniej stacji do pierwszej
        {
            c_station = 0;
        }
        else
        {
            c_station += 1;
        }         
    }


    void walk()
    {
        int destination;

        // Losowanie stacji, innej niz ta na ktorej obecnie jest
        while (true)
        {
            int nr = std::uniform_int_distribution<int>(0, STATIONS_NUMBER-1)(rng);

            if (nr != c_station)
            {
                destination = nr;
                break;
            } 
        }

        // Hardcodowanie sciezek:
        // Wzdluz torow (0, 1, 2, 3)
        if (c_station == 0 && destination == 1) { c_path = 0; }
        if (c_station == 1 && destination == 0) { c_path = 0; }
        if (c_station == 1 && destination == 2) { c_path = 1; }
        if (c_station == 2 && destination == 1) { c_path = 1; }
        if (c_station == 2 && destination == 3) { c_path = 2; }
        if (c_station == 3 && destination == 2) { c_path = 2; }
        if (c_station == 3 && destination == 0) { c_path = 3; }
        if (c_station == 0 && destination == 3) { c_path = 3; }
        // Na skos (4, 5)
        if (c_station == 0 && destination == 2) { c_path = 4; }
        if (c_station == 2 && destination == 0) { c_path = 4; }
        if (c_station == 1 && destination == 3) { c_path = 5; }
        if (c_station == 3 && destination == 1) { c_path = 5; }


        // Zmienna lokalna - kontrolowanie ilosci pieszych na sciezce
        std::unique_lock<std::mutex> path_lock(map.paths[c_path].mtx);     // Zablokowanie watku - sciezki - mutexu
        
        while (map.paths[c_path].passengers_number.load(std::memory_order_seq_cst) >= 2 && !end)   // Jesli na sciezce wiecej niz 2 pasazerow
        {
            map.paths[c_path].cond.wait(path_lock);                                               // czekaj (wait - atomowe zwolnienie mutexa i uspienie watku)
        }
        if (end)
        {
            return;
        }

        map.paths[c_path].passengers_number.fetch_add(1, std::memory_order_seq_cst);      // Gdy sciezka wolna idz i dodaj pasazera do sciezki

        path_lock.unlock();             // zwolnij mutex blokujacy podliczanie ludzi na sciezce
        cond.notify_one();              // wpusc kolejny watek (jesli czeka)

        std::shared_lock<std::shared_timed_mutex> path_lock2(map.paths[c_path].smtx);       // Zablokowanie sciezki - dzielonego mutexu

        state = 2;      // 2 - walk

        int time = std::uniform_int_distribution<int>(80, 100)(rng);

        for (auto i=1; i<=time; i++)
        {
            double p = (double) i / (double) time;
            progress = std::round(p * 100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));        // wstrzymanie wykonywania biezacego watku na 0.1 s

            if (end)
            {
                return;
            }
        }
        c_station = destination;

        //std::unique_lock<std::mutex> path_lock3(map.paths[c_path].mtx);
        map.paths[c_path].passengers_number.fetch_sub(1, std::memory_order_seq_cst);    // odejmij pasazera z liczby ludzi na sciezce

    }

};


// Wizualizacja symulacji w terminalu
class Visualisation
{
    private:
        int x;
        int y;
        int row;
        int col;

        std::vector<Train *> trains;
        std::vector<Passenger *> passengers;
        RailwayMap &map;

        std::map<int, const char *> train_states =
        {
            std::pair<int, const char *>(-1, "not ready"),
            std::pair<int, const char *>(0, "stopped"),
            std::pair<int, const char *>(1, "riding")
        };

        std::map<int, const char *> passenger_states =
        {
            std::pair<int, const char *>(-1, "not ready"),
            std::pair<int, const char *>(0, "waiting"),
            std::pair<int, const char *>(1, "riding"),
            std::pair<int, const char *>(2, "walking")
        };

    public:
        Visualisation(std::vector<Train *> trains, std::vector<Passenger *> passengers, RailwayMap &_map);
        ~Visualisation();
        void update();
};



#define PBAR "###################################################"
#define LENGTH 30


Visualisation::Visualisation(std::vector<Train *> trains, std::vector<Passenger *> passengers, RailwayMap &_map) : trains(trains), passengers(passengers), map(_map)
{
    initscr();
    noecho();
    raw();
    nodelay(stdscr, TRUE);
    start_color();
    use_default_colors();
    curs_set(0);
    init_pair(3, COLOR_RED, -1);
    init_pair(2, COLOR_GREEN, -1);
    init_pair(1, COLOR_CYAN, -1);
    getmaxyx(stdscr, col, row);
    x = row/2;
    y = col/2;
}

Visualisation::~Visualisation()
{
    endwin();
}


void Visualisation::update()
{
    printw("[ESC] - Escape the RailwaySimulatior\n\n");

    // Mapa I:
    printw("          . . . . . . .              LEGEND:                             PATHS:                    RAILS:      \n");
    printw("        .S0-----0-----S1.                                                                                      \n");
    printw("        .|  .       .  |.         -|  ->  rail                     Path 0: S0 <-> S1          Rail 0: S0 -> S1 \n");
    printw("        .|    .   .    |.        . .  ->  path                     Path 1: S1 <-> S2          Rail 1: S1 -> S2 \n");
    printw("        .3      .      1.         S0  ->  station 0                Path 2: S2 <-> S3          Rail 2: S2 -> S3 \n");
    printw("        .|    .   .    |.          0  ->  rail 0                   Path 3: S3 <-> S4          Rail 3: S3 -> S0 \n");
    printw("        .|  .        . |.                                          Path 4: S0 <-> S2                           \n");
    printw("         S3-----2-----S2                                           Path 5: S1 <-> S3                           \n");
    printw("          . . . . . . .                                                                                          ");


    // Mapa II:

    // printw("        S0-----0------S1-----1-----S2\n");
    // printw("        |  .          |        .... |\n");
    // printw("        |    . . . . 17........     |\n");
    // printw("        |       .     |    ..       |\n");
    // printw("        6         . .S7...          2\n");
    // printw("        |       ...   |    ..       |\n");
    // printw("        |    ........74........     |\n");
    // printw("        | ...         |        .... |\n");
    // printw("        S5-----4------S4-----3-----S3");
    

    WINDOW * w1 = newwin(TRAINS_NUMBER, 25, 12, 2);                     // Okno lokalizacji pociagu
    WINDOW * w2 = newwin(STATIONS_NUMBER, 25, 12, 32);                  // Okno wagonow
    WINDOW * w3 = newwin(PASSENGERS_NUMBER, 29, TRAINS_NUMBER + 16, 2); // Okno lokalizacji pasazerow
    WINDOW * w4 = newwin(PATHS_NUMBER, 25, TRAINS_NUMBER + 16, 32);     // Okno sciezek


    while (true)
    {
        int key = getch();
        if (key == 27)
        {
            for (auto train : trains)
            {
                train -> end = true;
            }
            for (auto passenger : passengers)
            {
                passenger -> end = true;
            }
            return;
        }
        
        for (auto train : trains)
        {
            int id = train -> id;
            int state = train -> state;
            int rail_id = train -> c_railF;
            int train_station_id = train -> c_station;
            

            // Okno 1: Dane pociagow
            for (int i=0; i<TRAINS_NUMBER; i++)
            {
                mvwprintw(w1, i, 1, "Train %d : ", i);
            }
            wrefresh(w1);

            // Okno 2: Dane wagonow na stacjach
            for (int i=0; i<STATIONS_NUMBER; i++)
            {
                mvwprintw(w2, i, 1, "Station %d: ", i);
            }
            wrefresh(w2);

            // Przesuniecie
            move(y + id, 0);
            move(y + id - 7, x - 8);

            clrtoeol();     // Wyczyszczenie okna
            

            if (state == 0)     // Gdy pociag stoi
            {   
                attron(COLOR_PAIR(3));
                printw("Train %d : %s", id, train_states[state]);
                mvwprintw(w1, id, 13, "Station %d", train_station_id);
                mvwprintw(w2, 0, 13, "%d carriages", map.carriages[0].free_carriages.load(std::memory_order_relaxed));
                mvwprintw(w2, 1, 13, "%d carriages", map.carriages[1].free_carriages.load(std::memory_order_relaxed));
                mvwprintw(w2, 2, 13, "%d carriages", map.carriages[2].free_carriages.load(std::memory_order_relaxed));
                mvwprintw(w2, 3, 13, "%d carriages", map.carriages[3].free_carriages.load(std::memory_order_relaxed));
                wclrtoeol(w1);
            }
            else                // Gdy pociag jedzie
            {
                int progress = train -> progress;
                int lpad = std::round(progress / 100.0f * LENGTH);
                int rpad = LENGTH - lpad;

                attron(COLOR_PAIR(state+1));
                printw("Train %d : %s %3d%% [%.*s%*s]", id, train_states[state], progress, lpad, PBAR, rpad, "");
                mvwprintw(w1, id, 13, "Rail %d", rail_id);
                mvwprintw(w2, 0, 13, "%d carriages", map.carriages[0].free_carriages.load(std::memory_order_relaxed));
                mvwprintw(w2, 1, 13, "%d carriages", map.carriages[1].free_carriages.load(std::memory_order_relaxed));
                mvwprintw(w2, 2, 13, "%d carriages", map.carriages[2].free_carriages.load(std::memory_order_relaxed));
                mvwprintw(w2, 3, 13, "%d carriages", map.carriages[3].free_carriages.load(std::memory_order_relaxed));
                wclrtoeol(w1);
            }
        }

        for (auto passenger : passengers)
        {
            int id = passenger -> id;
            int state = passenger -> state;
            int station = passenger -> c_station;
            int train = passenger -> c_train;
            int path = passenger -> c_path;

            // Okno 3: Dane pasazerow:
            for (int i=0; i<PASSENGERS_NUMBER; i++)
            {
                mvwprintw(w3, i, 1, "Passenger %d : ", i);
            }
            wrefresh(w3);

            for (int i=0; i<PATHS_NUMBER; i++)
            {
                mvwprintw(w4, i, 1, "Path %d : ", i);
            }
            wrefresh(w4);

            //Przesuniecie
            move(y + id, 0);
            move(y + id - 3 + TRAINS_NUMBER, x - 8);

            clrtoeol();     // Wyczyszczenie okna

            if (state == 0)     // Gdy pasazer czeka na stacji
            {
                attron(COLOR_PAIR(3));
                printw("Passenger %d : %s", id, passenger_states[state]);
                mvwprintw(w3, id, 16, "Station %d", station);
                mvwprintw(w4, 0, 11, "%d passengers", map.paths[0].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 1, 11, "%d passengers", map.paths[1].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 2, 11, "%d passengers", map.paths[2].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 3, 11, "%d passengers", map.paths[3].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 4, 11, "%d passengers", map.paths[4].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 5, 11, "%d passengers", map.paths[5].passengers_number.load(std::memory_order_relaxed));

                wclrtoeol(w3);
                wclrtoeol(w4);
                
            }
            if (state == 1)     // Gdy pasazer jedzie pociagiem
            {
                int progress = passenger -> progress;
                int lpad = std::round(progress / 100.0f * LENGTH);
                int rpad = LENGTH - lpad;

                attron(COLOR_PAIR(2));
                printw("Passenger %d : %s %3d%% [%.*s%*s]", id, passenger_states[state], progress, lpad, PBAR, rpad, "");
                mvwprintw(w3, id, 16, "Train %d", train);
                mvwprintw(w4, 0, 11, "%d passengers", map.paths[0].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 1, 11, "%d passengers", map.paths[1].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 2, 11, "%d passengers", map.paths[2].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 3, 11, "%d passengers", map.paths[3].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 4, 11, "%d passengers", map.paths[4].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 5, 11, "%d passengers", map.paths[5].passengers_number.load(std::memory_order_relaxed));

                wclrtoeol(w3);
                wclrtoeol(w4);
            }

            if (state == 2)     // Gdy pasazer idzie na piechote
            {
                int progress = passenger -> progress;
                int lpad = std::round(progress / 100.0f * LENGTH);
                int rpad = LENGTH - lpad;

                attron(COLOR_PAIR(1));
                printw("Passenger %d : %s %3d%% [%.*s%*s]", id, passenger_states[state], progress, lpad, PBAR, rpad, "");
                mvwprintw(w3, id, 16, "Path %d", path);
                mvwprintw(w4, 0, 11, "%d passengers", map.paths[0].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 1, 11, "%d passengers", map.paths[1].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 2, 11, "%d passengers", map.paths[2].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 3, 11, "%d passengers", map.paths[3].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 4, 11, "%d passengers", map.paths[4].passengers_number.load(std::memory_order_relaxed));
                mvwprintw(w4, 5, 11, "%d passengers", map.paths[5].passengers_number.load(std::memory_order_relaxed));

                wclrtoeol(w3);
                wclrtoeol(w4);
            }
        }
    }
}


int main() 
{
    std::vector<Train *> trains;            // Kontener obiektow klasy Train
    std::vector<Passenger *> passengers;    // Kontener obiektow klasy Passenger
    RailwayMap map;
    std::mt19937 rng{std::random_device{}()};
    

    
    // Pociagi startuja z losowych stacji
    for (auto i=0; i<TRAINS_NUMBER; i++)  
    {   
        int nr = std::uniform_int_distribution<int>(0, STATIONS_NUMBER-1)(rng);   // Randomowy start pociagu
        trains.push_back(new Train(i, std::ref(map), map.stations[nr].id, map.rails[nr].id));   
    }
    
    
    /*
    // Pociagi startuja z tej samej stacji
    int nr = std::uniform_int_distribution<int>(0, STATIONS_NUMBER)(rng);
    for (auto i=0; i<TRAINS_NUMBER; i++)   
    {   
        trains.push_back(new Train(i, std::ref(map), map.stations[nr].id, map.rails[nr].id));   // Utworzenie obiektow Train (przekazanie im argumentow i referencji)
    }
    */

   // Pasazerowie startuja z losowych stacji
   for (auto i=0; i<PASSENGERS_NUMBER; i++)
   {
       int nr = std::uniform_int_distribution<int>(0, STATIONS_NUMBER-1)(rng);        // Randomowa stacja startu pasazera
       passengers.push_back(new Passenger(i, std::ref(map), map.stations[nr].id));
   }



    map.ready = true;           // Po utworzeniu watkow, mapa jest gotowa

    // Stworzenie obiektu wizualizacji
    Visualisation v(trains, passengers, std::ref(map));
    std::thread th_v(&Visualisation::update, &v);       // Watek do wizualizacji
    th_v.join();                                        // Uruchomienie watku wizualizacji

    for (auto tr : trains)      // Uruchomienie watkow pociagow
    {
        tr -> th.join();
    }

    for (auto ps : passengers)  // Uruchomienie watkow pasazerow
    {
        ps -> th.join();
    }

    endwin();
    return 0;
}