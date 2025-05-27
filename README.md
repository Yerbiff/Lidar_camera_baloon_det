# Estymacja Pozycji Obiektu na Podstawie Danych z Kamery i Lidaru

## Opis projektu

Celem projektu było stworzenie systemu estymującego pozycję balonu w przestrzeni 3D na podstawie obrazu z kamery RGB. Jako dane referencyjne (ground truth) wykorzystano dane z lidaru.

Dane zostały nagrane w formacie rosbag z wykorzystaniem skalibrowanego zestawu kamera + lidar dla różnych odległości.

---

## Sprzęt i konfiguracja

- Kamera RGB (skalibrowana względem lidara)
- Lidar 3D
- Balon jako obiekt śledzenia
- Samochód z zamontowaną kamerą i lidarem
- ROS 2 + rosbag

---

## Etapy realizacji

### 1. Nagranie danych
- Balon był poruszany przed kamerą i lidarem.
- Nagrania wykonano z dystansów: **3 m**, **6 m**, **12 m**.
- Dane zapisano do plików `.bag`.

### 2. Przetwarzanie danych
- Z danych lidarowych wysegmentowano ręcznie punkty odpowiadające piłce (przy użyciu [Supervisely](https://app.supervisely.com/)).
- Obliczono środek obiektu w przestrzeni 3D — uznano go za ground truth.

### 3. Estymacja z kamery
- Wykorzystano klasyczne podejścia do estymacji pozycji balonu na podstawie obrazu (segmentacja koloru)

### 4. Porównanie wyników
- Porównano błędy estymacji pozycji względem ground truth z lidara.
- Przeanalizowano wpływ odległości na dokładność estymacji.

---

## Wnioski

---

## Autorzy

__JJayU__ - [Github](https://github.com/JJayU) (Jakub Junkiert, 147550)

__Yerbiff__ - [Github](https://github.com/Yerbiff) (Jarosław Kuźma, 147617)

__the_HaUBe__ - [Github](https://github.com/theHaUBe) (Hubert Górecki, 147599)
