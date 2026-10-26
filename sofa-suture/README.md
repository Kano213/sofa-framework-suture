# Simulation Chirurgicale de Suture — Version Finale Haute Fidélité (SOFA v26.06.00)

Ce projet implémente une simulation interactive 3D complète de suture cutanée sous **SOFA Framework** (`SofaPython3`), réunissant un modèle biomécanique par éléments finis corotationnels (`TetrahedralCorotationalFEMForceField`), une gestion multi-surfaces de la perforation chirurgicale et les **3 fonctionnalités avancées de fermeture de plaie** :

1. **Option 1 — Tension de Suture & Coaptation Élastique FEM des Berges** : Rapprochement progressif et régulé des lèvres de l'incision (de `2.0 mm` d'écart initial jusqu'à `0.0 mm`, contact franc sans croisement ni inversion).
2. **Option 2 — Nœud Chirurgical Permanent (Knot Tying)** : Création d'un nœud chirurgical plat à double boucle, pont arqué, brins coupés et orifices permanents ancrés par interpolation barycentrique sur la peau déformée.
3. **Option 3 — Suture Multi-Points (jusqu'à 6 points)** : Coupe automatique du fil et réarmement de l'aiguille le long de l'incision centrale de `4 cm` pour fermer l'intégralité de la plaie point par point.

---

## Lancement Rapide

Double-cliquez sur **[`lancer.bat`](file:///C:/Users/djaou/Desktop/sofa-suture-final/lancer.bat)** ou exécutez dans un terminal :

```powershell
& "C:\Users\djaou\SOFA\v26.06.00\bin\runSofa.exe" "C:\Users\djaou\Desktop\sofa-suture-final\skin_simulation.py"
```

> **Note importante dans l'interface SOFA** : Cliquez sur le bouton **Animate** (en haut à gauche) pour démarrer la boucle physique temps réel avant d'utiliser les commandes clavier/souris.

---

## Commandes Clavier & Souris

### 1. Manipulation de l'Aiguille Courbe
| Touche / Action | Fonction Chirurgicale |
| :--- | :--- |
| **`Flèches`** ou **`Z` `Q` `S` `D`** | Déplacer l'aiguille horizontalement (axes `X` / `Z`) |
| **`F`** ou **`C`** | **Descendre** l'aiguille (appuyer fermement contre la peau pour percer l'épiderme) |
| **`R`** | **Monter** l'aiguille (et assister la remontée de la pointe hors de la peau) |
| **`E`** | **Enrouler** l'aiguille le long de sa courbure (traversée des berges & émergence fluide) |
| **`A`** ou **`Retour`** | **Dérouler** l'aiguille en sens inverse (enroulement/déroulement sans limite) |
| **`J`** / **`L`** | Pivoter la pince porte-aiguille en **lacet** (Yaw gauche / droite) |
| **`I`** / **`K`** | Incliner la pince porte-aiguille en **tangage** (Pitch avant / arrière) |
| **`U`** | Réaligner l'orientation de la pince (`Yaw = 0°`, `Pitch = 0°`) |
| **`TAB`** | Activer / Désactiver le **guidage direct à la souris** (Clic gauche = Enrouler, Clic droit + glisser = Hauteur `Y`) |

### 2. Geste de Suture, Tension & Nœud Chirurgical
| Touche | Action |
| :--- | :--- |
| **`V`** ou **`Espace`** | **Serrer la tension du fil** : rapproche et coapte les deux berges de l'incision (`2.0 mm -> 0.0 mm`) |
| **`B`** | **Desserrer la tension** du point de suture actif |
| **`N`** | **Nouer le point de suture** : verrouille la coaptation, crée le nœud permanent à brins coupés et prépare le point suivant |
| **`T`** | **Assistance chirurgicale** : valide manuellement la percée d'entrée ou la contre-ponction de sortie |
| **`X`** | **Réinitialisation complète** : rouvre la plaie (`2.0 mm`) et efface tous les points noués |

---

## Améliorations Techniques de la Version Finale

- **Émergence 3D Garantie & Zéro Étirement Parasite** :
  - Désactivation complète du `SphereCollisionModel` (`active = False`) dès l'entrée dans le derme afin d'éliminer tout accrochage ou soulèvement parasite du maillage de peau lors de la contre-ponction de sortie.
  - Accompagnement cinématique automatique du centre de courbure vers sa hauteur d'arc chirurgicale optimale (`Y = 2.16 cm`) lors de l'enroulement (`[E]`) ou de la remontée (`[R]`), garantissant que la pointe ressorte physiquement et visuellement au-dessus de l'épiderme (`Y >= 1.63 cm`, soit `+1.3 mm` au-dessus de la surface `Y = 1.50 cm`).
- **Traversée Multi-Surfaces Symétrique (1 Temps, 2 Temps ou Suture Libre)** :
  - Prise en charge symétrique Gauche -> Droite et Droite -> Gauche, avec détection des parois internes de la fente (`exit_fente` et `entry_fente`) et ancrage barycentrique cohérent du fil dans jusqu'à 4 orifices par point.
- **Régulation de Coaptation Dermique Sans Chevauchement** :
  - Répartition de l'effort de fermeture sur le derme superficiel et profond avec amortissement en vitesse et barrière de contact, assurant une fermeture franche des lèvres de la plaie sans oscillation ni interpénétration.
