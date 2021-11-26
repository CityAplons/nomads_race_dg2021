# Команда NOMADS

## Лига: RACE, экспертная

> Статус: не доработан
>
> Было опробовано несколько методов, включая детекцию с помощью openCV,
>
> Результат не стабилен, работает только с одним дроном

### Установка

1. Разархивируйте проект в воркспейс ros

2. Установите необходимые пакеты:

    ```bash
    pip3 install open3d bezier scipy
    pip3 install airsim # Вернуть старую вверсию tornado
    ```

3. Соберите проект

    ```bash
    cd ~/catkin_ws && catkin build
    source ~/catkin_ws/devel/setup.bash
    ```

4. Запуск

    ```bash
    roslaunch dg_solutions race.launch drone_count:=N
    ```

    Где N - количество дронов.

Link to hackathon repo: [acsl-mipt/drone-games](https://github.com/acsl-mipt/drone-games.git)

## Docker commands

+ [__NOT REQUIRED__] Update docker image (will install latest changes from [acsl-mipt/drone-games](https://github.com/acsl-mipt/drone-games.git))

    ```bash
        docker-compose build
    ```

+ Create your own container

    ```bash
        docker-compose run --name [_CONTAINER NAME_] dronegames
    ```

    > You will be prompted to container bash after this command

+ Starting your container

    ```bash
        docker start [_CONTAINER NAME_]
    ```bash

+ Enter into a container (bash)

    ```bash
        docker exec -it [_CONTAINER NAME_] bash
    ```
