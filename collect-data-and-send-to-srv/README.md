zmien to później by ładnie wyglądało
## Kalibracja iSpindel

Proces kalibracji polega na sporządzeniu kilku roztworów o znanych gęstościach (°Plato / SG) i zmierzeniu kąta nachylenia iSpindla dla każdego z nich. Na podstawie pomiarów generuje się równanie kalibracyjne, które umożliwia bezpośredni odczyt gęstości w oprogramowaniu urządzenia.

| Punkt | SG     | °Plato | Woda (ml) | Cukier (g) | Uwagi |
|:------|:-------|:-------|:----------:|:-----------:|:------|
| 1 | 1.000 | 0° | 3000 | 0 | Czysta woda destylowana lub kranowa |
| 2 | 1.050 | ~12.5° | 2143 | 1000 | Zacznij od 3 L wody + 1 kg cukru, gotuj do rozpuszczenia |
| 3 | 1.040 | ~10° | 2347 | 1000 | Rozcieńcz punkt 2 o ~200 ml wody |
| 4 | 1.030 | ~7.5° | 2571 | 1000 | Rozcieńcz punkt 3 o ~224 ml wody |
| 5 | 1.020 | ~5° | 2857 | 1000 | Rozcieńcz punkt 4 o ~286 ml wody |
| 6 | 1.010 | ~2.5° | 3226 | 1000 | Rozcieńcz punkt 5 o ~369 ml wody |

### Obliczanie formuły kalibracyjnej

1. Zanotuj kąty nachylenia (`Tilt`) z iSpindla dla każdego roztworu.
2. Wprowadź dane (`Tilt` ↔ `SG` lub `°Plato`) do arkusza kalkulacyjnego.
3. Utwórz dopasowanie wielomianowe 2. lub 3. stopnia, np.:

\[
SG = a \cdot Tilt^3 + b \cdot Tilt^2 + c \cdot Tilt + d
\]

4. Otrzymane współczynniki \(a, b, c, d\) służą do obliczeń w firmware iSpindla.
5. W interfejsie konfiguracyjnym w sekcji **Calibration** wprowadź formułę dokładnie w tej postaci.

Przykładowa formuła (dla celów demonstracyjnych):

\[
SG = 0.0000007 \cdot Tilt^3 - 0.0004 \cdot Tilt^2 + 0.0925 \cdot Tilt + 0.830
\]

Po zapisaniu zmian iSpindel będzie przeliczać odczyty kąta na wartość SG w sposób automatyczny.
