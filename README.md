# zen_pid
Détails d’implémentation & raisons de design

Modèle discret minimal 
𝑦
𝑘
=
𝑎
𝑦
𝑘
−
1
+
𝑏
𝑢
𝑘
−
𝑑
y
k
	​

=ay
k−1
	​

+bu
k−d
	​

 : suffisant pour de très nombreux systèmes assimilables à FOPDT ; la conversion analytique 
𝑎
↦
𝜏
a↦τ, 
𝑏
↦
𝐾
b↦K élimine une étape d’optimisation non linéaire fragile. (cadre enseigné par Ljung). 
MIT BME

Estimation de délai par corrélation croisée sur fenêtre glissante : robuste au bruit, en ligne, O(W·D) (W = fenêtre, D = retards testés). On ajoute un raffinement quadratique (sous‑échantillon) pour gagner un demi‑pas d’échantillonnage sans FFT. (évaluations comparatives et revues méthodes TDE). 
RT Isy
+1

RLS à oubli adaptatif : on réduit 
𝜆
λ quand l’erreur normalisée ρ grandit → apprentissage très rapide lors de changements (procédé, latence, perturbations), puis 
𝜆
→
𝜆
max
⁡
λ→λ
max
	​

 quand la situation se calme → paramètres stables. (principe soutenu par la littérature RLS « variable forgetting »). 
dsbaero.engin.umich.edu

Tuning IMC/SIMC (Skogestad) : conversions fermées (pas d’optimisation) donnant d’excellents compromis rapidité/robustesse ; on impose 
𝜆
≥
0.8
𝐿
λ≥0.8L en pratique pour éviter l’acharnement face au délai. 
NTNU

Bouclier industriel : anti‑windup par rétro‑calcul, dérivée sur la mesure (filtrée), pondération de consigne (β), limites d’effort, bumpless via intégrateur, métriques (FIT%, RMSE), rapport JSON pour audit/traçabilité.

Latence : L reflète la somme des délais capteur‑bus‑calcul‑actionneur‑procédé. Si vous avez des time‑stamps précis côté acquisition et commande, on peut séparer la part « numérique » et la part « procédé » ; en l’état, pour le réglage c’est le délai effectif qui compte (et c’est ce que nous estimons). Références de fond sur contrôle de processus à délai. 
Frontiers

Conseils de mise en service (pour “apprendre très vite”)

Choisir 
𝑇
𝑠
T
s
	​

 tel que 
𝑇
𝑠
≤
𝜏
/
20
T
s
	​

≤τ/20 et 
𝑇
𝑠
≤
𝐿
/
5
T
s
	​

≤L/5 si possible (évite l’aliasing du délai).

Augmenter fast_adapt_gain (0.1–0.2) pour accélérer l’adaptation si vos résidus montent vite ; réduire lambda_min (0.85) en cas de dynamiques très changeantes. (Attention : plus agressif = plus de variance de paramètres.) 
dsbaero.engin.umich.edu

Fixer delay_max au‑delà de votre latence présumée (par ex. 0.5–1.0 s).

imc_lambda : commencez vers max(0.2·τ, 0.8·L) ; descendez si vous voulez plus de vitesse, remontez si bruit ou saturation. 
NTNU

Vérifier le rapport (JSON) : quand model.valid=true, les gains se recalculent automatiquement.

### Table de calibration persistante

Pour éviter de « réapprendre » après chaque redémarrage, `AdaptivePID` peut sauvegarder son état interne complet (modèle identifié, gains PID, historique de délai, intégrale, covariance RLS, etc.) dans un fichier texte compact :

```cpp
ctrl::AdaptivePID pid(Ts);
// ... exploitation en ligne ...
pid.saveCalibration("/chemin/vers/calibration.tbl");

ctrl::AdaptivePID pid_redemarrage(Ts);
pid_redemarrage.setOptions(opts);      // mêmes options qu’à l’apprentissage
pid_redemarrage.loadCalibration("/chemin/vers/calibration.tbl");
```

`captureCalibration()` renvoie également une structure `CalibrationSnapshot` manipulable en mémoire (utile pour persister dans une base de données ou transférer l’état sur le réseau). Le format de fichier versionné (`#AdaptivePID-calibration-v1`) garde la précision en double et reste lisible manuellement si besoin.

## Validation par simulation

Un environnement de test minimal est fourni dans `tests/pid_validation.cpp` pour observer le comportement du contrôleur sur un procédé FOPDT synthétique et vérifier la qualité des estimations (gain, constante de temps, latence) ainsi que les performances en boucle fermée.

### Suivi polynomial sur moteur inertiel

Pour valider l’apprentissage sur une dynamique « moteur + inertie » de second ordre, le programme `tests/test_motor_polynomial.cpp` simule un asservissement de position avec une consigne polynomiale bornée (accélération/décélération douce) et une perturbation d’accélération transitoire.

```bash
g++ -std=c++17 -O2 -I. tests/test_motor_polynomial.cpp -o tests/test_motor_polynomial
./tests/test_motor_polynomial
```

L’exécutable génère `tests/data/motor_polynomial_run.csv` contenant le chronogramme complet (consigne, position vraie/bruitée, commande, modèle identifié, FIT%) et vérifie automatiquement plusieurs critères :

- erreur quadratique moyenne < 0,05 rad sur 40 s,
- erreur moyenne sur les 5 dernières secondes < 0,05 rad (absence de biais stationnaire),
- erreur absolue maximale < 0,25 rad malgré la perturbation,
- modèle identifié cohérent (`K ≈ 0,68`, `τ ≈ 1,16 s`, `model.valid=true`, `FIT ≈ 95 %` avec les paramètres par défaut).

Le rapport console confirme la qualité du suivi et des paramètres appris ; il peut être archivé avec le CSV pour audit.

### Compilation & exécution rapides

```bash
cd tests
./run_validation.sh
```

Le script compile un exécutable `pid_validation` (g++ C++17) puis lance une simulation de 40 s avec des changements de consigne et une perturbation de charge. La sortie console résume :

- les paramètres réels du procédé utilisés pour générer les données,
- l’estimation finale fournie par `AdaptivePID` (y compris le délai en secondes et en nombre d’échantillons),
- les gains PID calculés,
- des métriques de suivi (FIT%, RMSE interne et erreur RMSE/MAE/IAE vs. consigne),
- une excitation PRBS initiale appliquée en feedforward pour accélérer l’identification (désactivable via `--no-excitation`).

### Options principales

L’exécutable accepte différentes options pour configurer le scénario :

```
  --K <valeur>          gain procédé (défaut 1.8)
  --tau <valeur>        constante de temps en s (défaut 1.2)
  --delay <valeur>      latence en s (défaut 0.35)
  --Ts <valeur>         période d’échantillonnage en s (défaut 0.05)
  --duration <valeur>   durée de simulation en s (défaut 40)
  --steps <valeur>      force un nombre fixe d’itérations (outrepasse --duration)
  --imc-lambda <valeur> lambda IMC cible (facultatif)
  --noise-std <valeur>  écart-type du bruit de mesure (défaut 0.01)
  --init-Kp <valeur>    gain proportionnel initial (défaut 0.5)
  --init-Ti <valeur>    temps intégral initial en s (défaut 1.0)
  --init-Td <valeur>    temps dérivatif initial en s (défaut 0.0)
  --umin <valeur>       saturation minimum de sortie (défaut -5)
  --umax <valeur>       saturation maximum de sortie (défaut +5)
  --excitation-duration <valeur> durée de l’excitation PRBS initiale (défaut 4 s)
  --excitation-amplitude <valeur> amplitude excitation (défaut 1.5)
  --excitation-period <valeur> période de mise à jour PRBS (défaut 0.5 s)
  --no-excitation       désactive l’excitation initiale
  --sp t:valeur         changement de consigne (répétable)
  --dist t:valeur       perturbation de charge additive (répétable)
  --csv fichier         export CSV détaillé
  --delay-max N         forcer delay_max (échantillons)
```

Exemple avec une consigne supplémentaire, plus de bruit et export CSV :

```bash
./run_validation.sh --sp 5:0.8 --noise-std 0.02 --csv validation.csv
```

Le fichier CSV contient le chronogramme complet (setpoint, mesure bruitée, commande, paramètres estimés, métriques FIT) pour analyse ultérieure (Python, Excel, etc.).

### Tracer les courbes de convergence

Par défaut, `run_validation.sh` enregistre le log complet dans `tests/data/latest_run.csv` (≈ 800 lignes pour la simulation de 40 s) et conserve la console dans `tests/data/sample_run.log` si vous la redirigez. Un exemple de capture est fourni dans `tests/data/sample_run.csv`.

Un script Python (`tests/plot_validation.py`) permet de tracer l’évolution de la consigne, de la sortie réelle/bruitée, des paramètres estimés (K, τ, L) et des gains PID (Kp, Ti, Td) à chaque itération. Il nécessite `matplotlib` :

```bash
python3 -m pip install --user matplotlib  # une seule fois
python3 tests/plot_validation.py tests/data/sample_run.csv
```

Ajoutez `--output chemin/figure.png` si vous souhaitez enregistrer une image ; par défaut, une fenêtre interactive permet d’inspecter les courbes de convergence itération après itération.

### Campagne multi-modèles (3000 itérations chacun)

Pour tester l’identification et la robustesse du PID sur « plein de systèmes », un script dédié génère une batterie de scénarios de 3000 itérations chacun, avec des profils de consigne et de perturbation propres à chaque modèle FOPDT :

```bash
python3 tests/run_validation_suite.py
```

La compilation est automatique ; le script enchaîne ensuite quatre procédés synthétiques (`fast_thruster`, `thermal_plate`, `slow_tank`, `precision_stage`) en respectant leurs périodes d’échantillonnage respectives. Les logs complets (3000 lignes par test) sont sauvegardés dans `tests/data/suite/<scenario>.csv` et un manifeste `tests/data/suite/suite_manifest.json` rappelle les paramètres utilisés (K, τ, L, consignes, perturbations, seed, bruit) ainsi que les métriques clés (RMSE, IAE, FIT%, gains estimés, etc.).

Chaque scénario démarre par une excitation PRBS courte pour favoriser l’apprentissage, puis rejoue ses changements de consigne personnalisés. Le manifeste facilite la traçabilité et permet de rejouer/visualiser n’importe quel cas avec `tests/plot_validation.py` si besoin (`python3 tests/plot_validation.py tests/data/suite/fast_thruster.csv`).

Extensions possibles (si vous le souhaitez plus tard)

Autotuning “relay test” (Åström‑Hägglund) comme pré‑amorçage des gains avant identification fine. 
slunik.slu.se
+1

Prédicteur de Smith si L est très grand et quasi constant. 
ScienceDirect

RLS multi‑facteurs d’oubli (paramètre‑dépendant) si K et τ varient à des rythmes différents. 
Clemson Engineering

Références essentielles (sélection courte)

Rivera, Morari, Skogestad — IMC → PID (règles analytiques). 
Skoge

Skogestad — SIMC (règles simples et robustes pour FOPDT). 
NTNU

Åström & Hägglund — PID Controllers: Theory, Design, and Tuning. 
UCG - Univerzitet Crne Gore

Ljung — System Identification: Theory for the User (RLS, validation). 
MIT BME

Mohseni & Bernstein — RLS à oubli variable (F‑test). 
dsbaero.engin.umich.edu

Björklund (Linköping) — évaluation corrélation croisée pour estimation de délai. 
RT Isy

Normey‑Rico (review) — contrôle des procédés à délai (Smith et variantes). 
Frontiers
