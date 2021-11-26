# Команда NOMADS

## Лига: RACE, экспертная

> Статус: не доработан, в процессе дебага планировщика пути
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
