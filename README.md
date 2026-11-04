# ROS 2 + Gazebo Harmonic docker

Repozytorium zawiera kod do wykorzystania przez studentów w celu realizacji zajęć z Percepcja i Sterowanie Robotów Autonomicznych (PiSRA) w semestrze zimowym 2026/2027.

## Wymagania

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#container-device-interface-cdi-support)
- [VS Code devcontainer plugin](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container)

> [!IMPORTANT]  
> System operacyjnym Ubuntu jest wymagany ze względu na obsługę GUI (wymaga innego podejścia przy wykorzystaniu Windowsa).

## Start

Otwórz VS Code w katalogu z projektem.
Przejdź do lewego dolnego rogu i kliknij niebieską ikonę z dwiema strzałkami skierowanymi do siebie. Z rozwijanego menu wybierz **"Open Folder in Container... ”** i poczekaj, aż docker się zbuduje. Może to potrwać do 10 minut przy wolniejszym połączeniu internetowym.

> [!TIP]
> Dla osób korzystających z Windowsa oraz WSL 2 przygotowano `Dockerfile.windows` oraz `compose.windows.yaml`. 

Po zalogowaniu się do dockera będzie on działał w sposób podobny do uruchamiania ROS na komputerze hosta. Wszystkie aplikacje GUI będą korzystać z domyślnego menedżera okien hosta, będziesz mieć również dostęp do wszystkich urządzeń na hoście, a także akceleracji GPU.
Docker ma preinstalowany [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials.html) i większość potrzebnych zależności oraz symulator [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/).

## Dwie osoby na jednym komputerze

Każda osoba pracuje we własnym kontenerze (wspólny jest tylko obraz `psra-ros2-jazzy`, budowany raz).

1. Każda osoba klonuje repozytorium do **własnego** katalogu, np. `~/psra_ania` i `~/psra_bartek`.
2. W swoim klonie tworzy plik `.devcontainer/.env` na podstawie `.devcontainer/.env.example` (plik nie jest commitowany):

   ```bash
   cp .devcontainer/.env.example .devcontainer/.env
   ```

   i ustawia **różne** wartości:

   | Zmienna         | Osoba 1  | Osoba 2  | Do czego służy                          |
   |-----------------|----------|----------|-----------------------------------------|
   | `DEV_NAME`      | `ania`   | `bartek` | nazwa kontenera (`psra-<DEV_NAME>`)     |
   | `ROS_DOMAIN_ID` | `11`     | `22`     | izolacja topiców ROS 2                  |
   | `GZ_PARTITION`  | `ania`   | `bartek` | izolacja komunikacji Gazebo             |

3. Otwiera swój klon w VS Code i wybiera **"Reopen in Container"**.

> [!WARNING]
> Kontenery działają w sieci hosta (`network_mode: host`). Bez różnych `ROS_DOMAIN_ID` i `GZ_PARTITION` obie osoby widziałyby nawzajem swoje topici i symulacje.

Sprawdzenie w kontenerze: `echo $ROS_DOMAIN_ID $GZ_PARTITION`, a na hoście: `docker ps` (dwa kontenery `psra-<DEV_NAME>-workspace-1`).

## Pierwsze uruchomienie

Dla osób, które nie miały doczynienia ze środowiskiem ROS 2 + Gazebo, zachęcam do przerobienia tutorialu: [Gazebo Tutorial](https://gazebosim.org/docs/harmonic/tutorials/). Pozwoli to zaznajomić się z tym środowiskiem i tworzyć w przyszłości zaawansowane symulacje.

Następnie pomocne będzie odpowiednia kontrola robotami w środowisku symulacyjnym, na dobry start proszę zaznajomić się z repozytorium: [Gazebo ROS 2 Control](https://github.com/ros-controls/gz_ros2_control/).

Na sam koniec pewnym podsumowaniem, a także praktycznym podejściem do tematu jest dostarczony od [Husariona](https://husarion.com/tutorials/ros2-tutorials/1-ros2-introduction/) tutorial dla ich kilku robotów.

> [!IMPORTANT] 
Należy pamiętać, aby po zbudowaniu wywołać komendę lub pracować w nowym terminalu:
>
> ```bash
> source ~/.bashrc
> ```
>
> W tym pliku dodane są już dwie ważne ścieżki:
>
> ```bash
> /opt/ros/$ROS_DISTRO/setup.bash
> /home/developer/ros2_ws/install/setup.bash
> ```

## Przykład
1. Zbuduj obszar roboczy wraz z simple_example package.  
2. Uruchom launcha `example.launch.py`, pokazujący, w jaki sposób należy połączyć Gazebo z ROS 2, aby możliwa była wzajemna komunikacja.


## Dodatkowe materiały
* [Getting Started](docs/getting_started.md)
* [ROS 2 Command Cheat Sheet](docs/cheatsheet.md)
* [ROS 2 Example packages in Python](docs/example.md)
* [Bridge communication between ROS and Gazebo](docs/ros_gz_bridge.md)
* [TF dla kamery i lidaru](docs/tf.md)