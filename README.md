# Estymacja Pozycji Obiektu na Podstawie Danych z Kamery i Lidaru

## Opis projektu

Celem projektu było stworzenie systemu estymującego pozycję balonu w przestrzeni 3D na podstawie obrazu z kamery RGB. Jako dane referencyjne (ground truth) wykorzystano dane z lidaru.

Dane zostały nagrane w formacie rosbag z wykorzystaniem skalibrowanego zestawu kamera + lidar dla różnych odległości.

---

## Sprzęt i konfiguracja

- Kamera RGB (skalibrowana względem lidara)
- Lidar 3D
- Zielona piłka jako obiekt śledzenia
- Samochód z zamontowaną kamerą i lidarem
- ROS 2 + rosbag

---

## Etapy realizacji

### 1. Nagranie danych
- Balon był poruszany przed kamerą i lidarem.
- Nagrania wykonano z dystansów: **3 m**, **6 m**, **12 m**, **20 m**, **29 m**.
- Dane zapisano do plików `.bag`.

### 2. Przetwarzanie danych
- Z danych lidarowych wysegmentowano ręcznie punkty odpowiadające piłce (przy użyciu [Supervisely](https://app.supervisely.com/)).
- Obliczono środek obiektu w przestrzeni 3D — uznano go za ground truth.

### 3. Estymacja z kamery
- Wykorzystano klasyczne podejścia do estymacji pozycji balonu na podstawie obrazu (segmentacja koloru)

### 4. Porównanie wyników

Porównano pozycję piłki estymowaną z obrazu z kamerą z ground truth pozyskanym z oznaczonych danych z lidara. Zbadano wpływ odległości na dokładność estymacji w przestrzeni XYZ.

| Odległość lidar [m] | Odległość kamera [m]  | Średni błąd XYZ [m] | Odch. standardowe [m]  |
|:-------------------:|:---------------------:|:-------------------:|:----------------------:|
|        4.1          |         4.0           |        0.37         |         0.08           |
|         7           |         6.4           |        0.70         |         0.12           |
|        13           |         11            |        2.20         |         0.18           |
|        21           |         16.5          |        4.88         |         0.25           |
|        29           |        N/A*            |        N/A*         |         N/A*           |

> \*Dla 29 m piłka nie została wykryta – prawdopodobnie przez cień lub zmianę koloru.

#### Wnioski:
- Dokładność estymacji pozycji z obrazu **pogarsza się wraz ze wzrostem odległości** od kamery.
- **Główna część błędu pochodzi z osi Z (głębi)** – błąd w płaszczyźnie XY był niewielki, i **mogł również wynikać z niedokładności ręcznego oznaczania pozycji piłki**.
- Dla większych odległości dokładność spada – wpływają na to ograniczenia rozdzielczości, oświetlenia i zniekształcenia optyczne.

---


## Autorzy

__JJayU__ - [Github](https://github.com/JJayU) (Jakub Junkiert, 147550)

__Yerbiff__ - [Github](https://github.com/Yerbiff) (Jarosław Kuźma, 147617)

__the_HaUBe__ - [Github](https://github.com/theHaUBe) (Hubert Górecki, 147599)
