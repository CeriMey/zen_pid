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
