# Concevoir les opérations booléennes de ngk

## Objet de ce document

Ce document est un guide d'étude et de conception en français pour les opérations booléennes de ngk. Il synthétise les idées nécessaires du chapitre 3 de *Geometric and Solid Modeling: An Introduction* de Christoph M. Hoffmann, puis les traduit dans l'architecture GMap et NURBS de ngk.

Ce n'est ni une traduction du chapitre, ni une spécification déjà adoptée par le projet. Les passages intitulés **Direction proposée pour ngk** sont des conclusions d'architecture tirées du livre et de l'état actuel du code.

L'objectif est de pouvoir répondre précisément à cette question :

> Comment passer de deux solides B-rep qui se rencontrent à un nouveau solide topologiquement cohérent représentant leur union, leur intersection ou leur différence ?

La réponse courte est :

```text
détecter
    -> construire des contacts canoniques
    -> découper les deux frontières de façon synchronisée
    -> classifier les fragments
    -> sélectionner les fragments utiles
    -> les assembler et les coudre
    -> valider le résultat
```

Le calcul des intersections n'est donc que le début de l'algorithme.

---

## 1. Que faut-il lire avant le chapitre sur les booléens ?

### Lecture indispensable avant le chapitre 3

Dans le livre de Hoffmann :

1. **Section 2.2.2 - Regularized Boolean Operations**, autour de la page 22.
   Elle explique pourquoi un noyau solide utilise des opérations booléennes régularisées.
2. **Section 2.3 - Boundary Representations**, pages 36 à 46.
   Elle introduit la séparation entre topologie et géométrie, les orientations, les arêtes, les faces et les composantes de surface.
3. **Section 2.4 - Topological Validity of B-rep Solids**, pages 46 à 61.
   Elle précise ce que signifie « être un solide valide » et distingue validité topologique et validité du plongement géométrique.

Dans la documentation de ngk :

1. `docs/topology_orientation_refactor.md` pour comprendre l'identité stable et l'orientation contextuelle.
2. `docs/model_api.md` pour comprendre la séparation entre `Model`, opérations, résultats et politiques de payload.
3. Dans la référence GMap privée, les sections sur les cellules, les opérations `sew`/`unsew`, le plongement et la couture géométrique.

Sans ces prérequis, le chapitre 3 peut donner l'impression que le booléen est essentiellement une succession d'intersections. En réalité, l'essentiel du travail consiste à préserver une frontière orientée et cohérente pendant sa transformation.

### Lecture indispensable avant d'implémenter sérieusement

Lire ensuite le **chapitre 4 - Robust and Error-Free Geometric Operations**, pages 111 à 151.

Il peut être lu après une première lecture conceptuelle du chapitre 3, mais il doit être lu avant de considérer l'algorithme prêt pour des entrées générales. Il explique notamment que :

- une tolérance ne rend pas automatiquement un prédicat cohérent ;
- la coïncidence approximative n'est pas transitive ;
- deux calculs équivalents peuvent conduire à des décisions topologiques opposées ;
- recalculer indépendamment le même événement sur deux faces adjacentes est dangereux ;
- la précision nécessaire dépend du conditionnement du problème.

### Lecture nécessaire pour les surfaces courbes et NURBS

Pour ngk, ces lectures sont presque aussi importantes que le chapitre 3 :

- **Section 5.7 - Edge Identification**, pages 193 à 203 : identité d'une portion de courbe, branches, courbes fermées, singularités et orientation.
- **Chapitre 6 - Surface Intersections**, pages 205 à 254 : suivi des courbes d'intersection, prédiction-correction, Newton, SVD, singularités et intersection de surfaces paramétriques.

Le chapitre 3 présente principalement le cas polyédrique. Son architecture reste pertinente pour les NURBS, mais ses primitives géométriques ne suffisent pas telles quelles.

### Lecture facultative à court terme

Le chapitre 7 sur les bases de Gröbner est intéressant pour l'implicitation, les singularités, les offsets et certains systèmes algébriques. Il n'est pas nécessaire pour construire la première architecture booléenne NURBS de ngk.

### Ordre de lecture conseillé

```text
2.2.2 -> 2.3 -> 2.4 -> chapitre 3
                            |
                            +-> chapitre 4 avant l'implémentation robuste
                            +-> 5.7 et chapitre 6 pour les NURBS
```

---

## 2. Vocabulaire minimal

### Solide B-rep

Un solide B-rep est décrit par sa frontière. Cette description contient deux couches :

- une **topologie**, qui dit quelles cellules sont incidentes ou adjacentes ;
- une **géométrie**, qui plonge les sommets, arêtes et faces dans l'espace.

Dans ngk, la topologie est portée par le `GMap` et les attributs géométriques sont attachés aux cellules identifiées par `VertexKey`, `EdgeKey`, `FaceKey`, etc.

### Support géométrique et domaine limité

Une face n'est pas une surface entière. C'est une région limitée d'une surface support.

Une arête n'est pas nécessairement une courbe entière. C'est une portion orientée d'une courbe support.

Pour une face NURBS, la frontière doit pouvoir être comprise dans deux espaces :

- comme courbe 3D sur le solide ;
- comme pcurve 2D dans le domaine paramétrique de la surface.

### Coquille

Une coquille, ou *shell*, est une composante connexe de la frontière. Un solide peut contenir :

- une coquille extérieure ;
- des coquilles intérieures délimitant des cavités ;
- éventuellement plusieurs composantes solides disjointes selon le modèle choisi.

### Orientation

Une frontière orientée permet de distinguer localement l'intérieur de l'extérieur.

Dans ngk :

- la géométrie possède son orientation paramétrique propre ;
- l'attribut d'une cellule possède une orientation de référence ;
- une vue topologique transporte l'orientation contextuelle donnée par son dart.

Il ne faut pas confondre ces trois niveaux.

### Booléen régularisé

Une opération ensembliste brute peut produire des morceaux de dimension inférieure, par exemple une face isolée ou une arête seule résultant du contact de deux volumes.

Une opération régularisée conserve l'intérieur volumique puis reprend sa fermeture. Son résultat reste ainsi dans le domaine des solides réguliers.

Conséquence pratique : un simple contact tangent ou deux faces opposées exactement superposées peuvent ne produire aucune frontière dans le résultat final, même si l'intersection ensembliste brute n'est pas vide.

---

## 3. Le vrai problème posé par un booléen

Considérons deux solides `A` et `B`.

Pour calculer `A ∩ B`, il ne suffit pas de connaître les courbes où leurs surfaces se coupent. Il faut construire une nouvelle frontière comprenant :

- les fragments de faces de `A` situés à l'intérieur de `B` ;
- les fragments de faces de `B` situés à l'intérieur de `A` ;
- les arêtes d'intersection qui relient ces fragments ;
- des sommets et adjacences identiques des deux côtés de chaque contact ;
- des coquilles fermées et correctement orientées.

Pour l'union et la différence, le calcul géométrique des contacts est essentiellement le même. Ce sont surtout les règles de sélection et d'orientation des fragments qui changent.

Cela conduit à séparer trois problèmes :

1. **Préparer une subdivision commune** des frontières.
2. **Décider quels fragments appartiennent au résultat**.
3. **Assembler ces fragments en une nouvelle topologie valide**.

Cette séparation est fondamentale pour l'architecture de ngk.

---

## 4. Représentation exigée par l'algorithme

Le chapitre de Hoffmann suppose une structure permettant de retrouver efficacement :

- depuis un sommet : ses arêtes et faces incidentes ;
- depuis une arête : ses sommets et ses faces incidentes, dans leur ordre autour de l'arête ;
- depuis une face : ses cycles de frontière orientés ;
- depuis une coquille : ses faces et son orientation ;
- depuis un solide : ses coquilles et leurs relations de contenance.

Une GMap peut exprimer ces relations par les orbites et les involutions. La traduction pour ngk ne doit toutefois pas exposer ces détails à l'algorithme haut niveau. Le booléen devrait utiliser autant que possible :

- `solid.sheets()` ou l'équivalent pour parcourir les coquilles ;
- `sheet.faces()` ;
- `face.edges()` et ses boucles orientées ;
- `edge.vertices()` et `edge.faces()` ;
- les vues orientées construites depuis le dart réellement atteint.

Les opérations de découpage et de couture peuvent ensuite descendre vers `TopologyEdit`.

### Irréductibilité des décisions, pas duplication des vérités

Le livre recommande de limiter les données géométriques redondantes, car deux représentations calculées séparément d'un même objet peuvent se contredire.

Pour ngk, cela ne signifie pas nécessairement stocker le minimum absolu. Les pcurves, par exemple, sont nécessaires. Cela signifie plutôt :

- une source d'autorité clairement définie pour chaque information ;
- des données dérivées identifiées comme telles ;
- une validation explicite des relations entre courbe 3D, pcurves, sommets et surface support.

---

## 5. Vue d'ensemble du pipeline proposé pour ngk

```text
1. Résoudre les opérandes
2. Construire les candidats de contact
3. Calculer et canoniser les contacts
4. Construire le graphe d'intersection commun
5. Découper les deux opérandes dans une transaction
6. Classifier les fragments
7. Sélectionner selon l'opération
8. Réconcilier et coudre la frontière retenue
9. Reconstruire les coquilles et solides
10. Valider puis valider la transaction
```

Chaque phase doit avoir un résultat inspectable. Un booléen ne devrait pas être une fonction opaque qui mélange immédiatement calcul numérique, mutation topologique et suppression de cellules.

---

## 6. Phase 1 - Résoudre et figer les opérandes

Les deux opérandes doivent être présents dans le même `Model` ou dans une carte de travail transactionnelle.

Il faut collecter pour chaque opérande :

- ses sommets ;
- ses arêtes ;
- ses faces ;
- ses coquilles ;
- les boîtes englobantes nécessaires ;
- une version ou un moyen de détecter qu'un plan est devenu obsolète.

Le code actuel possède déjà `BooleanOperand`, `OperandCells` et la détection d'un `StalePlan`. C'est une bonne base.

### Import d'un outil externe

Lorsqu'un solide outil vient d'une autre carte, son import doit faire partie de la même transaction que la préparation. Sinon, un échec peut laisser une copie partielle dans le modèle.

`prepare_boolean_with_external_tool` suit déjà cette idée.

---

## 7. Phase 2 - Réduction des paires candidates

Tester toutes les paires de faces donne un coût quadratique, même lorsque les solides ne se touchent presque pas.

La première passe doit donc construire des paires candidates par intersection de boîtes englobantes :

```text
faces de A = rouges
faces de B = bleues
ne produire que les paires rouge-bleu dont les boîtes se chevauchent
```

Les boîtes doivent être élargies par une tolérance de broad phase. Une paire rejetée ne sera jamais examinée ensuite ; il vaut donc mieux quelques faux positifs qu'un faux négatif.

### Direction proposée pour ngk

Commencer avec une structure simple :

- AABB de chaque face ;
- tri suivant l'axe dominant ou sweep simple ;
- requêtes uniquement entre les deux opérandes.

Un BVH ou un arbre de patches NURBS pourra venir ensuite. Il n'est pas nécessaire d'implémenter immédiatement les arbres d'intervalles et de segments exacts décrits historiquement par Hoffmann.

La broad phase ne décide jamais qu'il y a contact. Elle décide uniquement quelles paires méritent un calcul étroit.

---

## 8. Phase 3 - Contacts géométriques canoniques

La narrow phase calcule les contacts réels. Les familles utiles sont déjà proches de `BooleanContact` :

- point isolé ;
- courbe d'intersection ;
- recouvrement d'arêtes ;
- région de faces coïncidentes.

Une courbe de contact entre deux faces courbes devrait au minimum transporter :

- une représentation ou approximation 3D ;
- une pcurve sur la première face ;
- une pcurve sur la seconde face ;
- des paramètres cohérents aux extrémités ;
- son caractère ouvert ou fermé ;
- la précision obtenue ;
- les événements particuliers : tangence, branchement, singularité, contact de frontière.

### Règle de cohérence la plus importante

Un même événement géométrique doit être construit une seule fois, puis partagé par tous ses consommateurs.

Mauvaise organisation :

```text
la face f1 calcule son point sur l'arête e
la face f2 recalcule indépendamment le même point sur e
```

Bonne organisation :

```text
un ContactId représente l'événement sur e
f1, f2 et l'autre opérande référencent ce même événement
```

Cette règle évite qu'un point soit considéré sur une arête depuis une face, mais légèrement en dehors depuis la face adjacente.

### Graphe d'intersection commun

Les contacts doivent être organisés en un graphe :

- noeuds : points canoniques, sommets existants ou nouveaux ;
- arcs : portions ordonnées de courbes d'intersection ;
- incidences : faces et arêtes sources touchées ;
- orientation : sens sur chaque face et dans chaque pcurve.

Il faut pouvoir représenter temporairement un graphe incomplet. Une courbe rencontrée sur une première paire de faces peut être subdivisée plus tard par une autre paire.

---

## 9. Les cas locaux à comprendre

Le chapitre classe les contacts ponctuels selon les cellules qui se rencontrent. Pour des frontières 2D dans l'espace 3D, il existe six familles principales.

### 9.1 Face / face

Cas transversal : les supports se coupent suivant une courbe. Une portion de cette courbe se trouve dans les domaines limités des deux faces.

Actions :

- construire la courbe 3D et les deux pcurves ;
- déterminer les portions réellement contenues dans les deux trims ;
- orienter l'arc sur chacune des faces ;
- analyser ses extrémités, qui peuvent tomber sur des arêtes ou sommets.

Cas coïncident : les supports et les régions se recouvrent.

Actions :

- calculer l'intersection des domaines de trim ;
- comparer les orientations ;
- reporter une région coïncidente, pas seulement quelques points échantillonnés.

### 9.2 Arête / face

Cas transversal : un point coupe l'intérieur de l'arête et l'intérieur de la face.

Actions :

- subdiviser l'arête ;
- placer le point dans le graphe de trim de la face ;
- déterminer quel côté de l'arête entre dans l'autre solide ;
- transférer l'incidence aux faces adjacentes à l'arête.

Cas dégénéré : l'arête repose dans la surface support de la face.

Il faut alors déterminer quels sous-segments appartiennent au domaine de la face et quelles faces incidentes entourent localement du volume utile.

### 9.3 Arête / arête

Cas transversal : un point subdivise les deux arêtes.

Cas dégénéré : les arêtes partagent un intervalle.

Le recouvrement doit être synchronisé : mêmes extrémités, mêmes sous-segments et correspondance d'orientation sur les deux opérandes.

### 9.4 Sommet / face

Le sommet doit être inséré dans le graphe de la face. Les courbes issues des faces adjacentes au sommet doivent être raccordées à ce même noeud canonique.

### 9.5 Sommet / arête

L'arête est subdivisée au sommet existant. Il faut ensuite analyser les secteurs ou coins volumiques autour du sommet pour déterminer quelles branches entrent dans l'autre solide.

### 9.6 Sommet / sommet

Les deux voisinages complets se rencontrent. C'est le cas ponctuel le plus riche : plusieurs arêtes et faces peuvent devoir être identifiées, séparées ou rejetées.

Une simple fusion des coordonnées ne suffit pas. Il faut classifier les cônes locaux de matière autour du point.

### Pourquoi ces six cas comptent encore avec des NURBS

Les formules polyédriques changent, mais la dimension topologique des cellules en contact ne change pas. Cette classification reste donc une bonne structure de dispatch pour ngk.

---

## 10. Analyse du voisinage local

L'analyse du voisinage répond à cette question :

> Autour d'un point ou d'une portion de contact, quels secteurs appartiennent à l'intérieur de `A`, à l'intérieur de `B`, et donc au résultat ?

Pour une intersection transversale simple de deux faces, les normales permettent de déterminer les quatre secteurs locaux.

Pour une arête non-manifold ou un sommet, il peut exister plus de quatre secteurs. Il faut alors connaître l'ordre cyclique des faces autour de l'arête ou construire une section locale autour du sommet.

### MVP manifold pour ngk

Pour une première version limitée aux solides manifold fermés :

- une arête de frontière a normalement deux faces incidentes ;
- le voisinage d'un point régulier de face est un demi-espace intérieur ;
- le voisinage d'une arête est un coin volumique ;
- le voisinage d'un sommet peut être analysé par ses faces et arêtes incidentes orientées.

Cette restriction réduit fortement les cas, tout en gardant une architecture extensible vers le non-manifold.

### Extension non-manifold

Le `GMap` peut représenter des structures plus générales que des manifolds. Dans ce cas, le booléen doit raisonner en termes de secteurs de matière et d'incidences, pas supposer « deux faces par arête ».

Cette extension devrait être explicite dans les options ou les capacités annoncées de l'opération.

---

## 11. Phase 4 - Découpage synchronisé

Une fois le graphe de contact construit, il faut l'imprimer sur les deux frontières.

### Ordre recommandé

1. Regrouper et canoniser tous les points destinés à une même arête.
2. Les ordonner dans le domaine paramétrique de l'arête.
3. Découper chaque arête une seule fois selon la liste complète.
4. Réécrire les imprints de face avec les nouveaux sommets et arêtes.
5. Construire les subdivisions complètes des faces.
6. Produire la lineage source -> fragments.

Découper immédiatement à chaque découverte oblige à rechercher sans cesse quel fragment représente encore la source. Un plan global suivi d'une application transactionnelle est plus simple à raisonner.

Le code actuel suit déjà partiellement cette direction avec `edge_points`, `face_imprints`, `BooleanLineage` et `apply_boolean_splits`.

### Face temporairement incomplète

Un arc d'intersection isolé ne divise pas nécessairement immédiatement une face en régions valides. Plusieurs paires de faces peuvent ajouter successivement des arcs au même domaine UV.

Il faut donc distinguer :

- l'accumulation d'un graphe d'imprint ;
- la polygonisation ou extraction finale des cycles ;
- la création des nouvelles `FaceKey`.

### Couture GMap après subdivision

Une couture de dimension 3 identifie deux faces et, par conséquence, leurs arêtes et sommets correspondants. La référence GMap exige que les deux faces à coudre soient isomorphes.

Donc :

```text
intersecter -> synchroniser les subdivisions -> vérifier la correspondance -> 3-sew
```

et non :

```text
intersecter approximativement -> tenter de coudre des faces différentes
```

Lors de la couture, les attributs et plongements identifiés doivent être réconciliés par une politique explicite. La transaction de ngk est le bon endroit pour appliquer cette politique après validation topologique.

---

## 12. Phase 5 - Classification des fragments

Après subdivision, chaque fragment de face doit recevoir une relation par rapport à l'autre solide.

Une classification utile ne devrait pas se limiter à `Inside` et `Outside` :

```rust
enum FragmentClass {
    Inside,
    Outside,
    OnSameOrientation,
    OnOppositeOrientation,
    Uncertain,
}
```

Les deux classes `On...` sont nécessaires pour les faces coïncidentes. `Uncertain` doit provoquer une stratégie de résolution ou un échec explicite, pas une sélection arbitraire.

### Comment obtenir la classification

Il existe deux sources complémentaires.

#### Classification locale

Autour d'une courbe ou d'un point de contact, l'analyse du voisinage indique directement quels fragments sont à l'intérieur de l'autre opérande.

#### Propagation par adjacence

Une fois quelques fragments classifiés, la classification peut être propagée aux fragments voisins qui ne traversent aucune frontière de l'autre solide.

Cette propagation est essentielle : certaines faces entièrement intérieures ne touchent aucune courbe d'intersection.

#### Cas sans intersection de frontière

Si aucune paire de faces ne se coupe, il reste plusieurs possibilités :

- `A` est dans `B` ;
- `B` est dans `A` ;
- ils sont disjoints ;
- une configuration multi-coquilles combine plusieurs de ces relations.

Il faut alors effectuer un test de contenance par coquille ou composante, et non conclure automatiquement que le résultat est vide.

---

## 13. Phase 6 - Sélection selon l'opération

Une fois chaque fragment classifié, l'opération booléenne devient principalement une politique de sélection.

Pour les fragments non coïncidents :

| Source du fragment | Classe par rapport à l'autre solide | Intersection | Union | Différence `A - B` |
|---|---:|---:|---:|---:|
| `A` | Inside | garder | rejeter | rejeter |
| `A` | Outside | rejeter | garder | garder |
| `B` | Inside | garder | rejeter | garder et inverser |
| `B` | Outside | rejeter | garder | rejeter |

Cette table donne l'intuition principale. Les contacts coïncidents demandent des règles supplémentaires :

- même orientation : conserver une seule copie si elle appartient à la frontière du résultat ;
- orientation opposée : souvent annuler les deux copies pour une opération régularisée ;
- éviter dans tous les cas deux faces géométriquement identiques mais topologiquement concurrentes.

### Séparer sélection et mutation

Il est préférable de construire d'abord une décision pure :

```text
FragmentDecision {
    source,
    fragment,
    class,
    action: Keep | KeepReversed | Drop | MergeWith(...)
}
```

Puis d'appliquer toutes les décisions ensemble. Cela rend possible :

- un mode preview ;
- des diagnostics lisibles ;
- des tests sans mutation ;
- une application atomique.

---

## 14. Phase 7 - Assemblage de la frontière

Les fragments retenus doivent devenir une ou plusieurs coquilles fermées.

L'assemblage comprend :

1. réutiliser ou créer les sommets canoniques ;
2. identifier les arêtes représentant le même segment de contact ;
3. faire correspondre leurs orientations ;
4. coudre les faces compatibles ;
5. parcourir les composantes connexes obtenues ;
6. déterminer leur orientation et leur relation de contenance ;
7. enregistrer les coquilles extérieures et intérieures ;
8. créer les nouveaux `SolidKey`.

### Identité et lineage

Un booléen détruit l'idée naïve qu'une face source survit toujours comme la même entité. Une face peut :

- disparaître ;
- survivre inchangée ;
- être découpée en plusieurs fragments ;
- être fusionnée avec une face coïncidente ;
- changer de coquille ;
- être conservée avec orientation inversée.

Le résultat doit donc exposer une lineage riche, par exemple :

```rust
struct BooleanResult {
    solids: Vec<SolidKey>,
    kept_faces: Vec<FaceKey>,
    created_faces: Vec<FaceKey>,
    removed_faces: Vec<FaceKey>,
    source_to_faces: HashMap<FaceKey, Vec<FaceKey>>,
    contacts: Vec<BooleanContactId>,
}
```

Le détail exact reste à décider, mais le résultat ne devrait pas être seulement un `SolidKey`.

### Payloads

Quand deux cellules deviennent une seule cellule, il faut décider quel payload survit ou comment les deux sont fusionnés. Quand une cellule est séparée, il faut décider comment son payload est copié ou transformé.

Cette décision appartient à une `EditPolicy` ou une politique booléenne, pas à un `HashMap` d'attributs manipulé directement depuis le builder.

---

## 15. Multi-coquilles et cavités

Une cavité est représentée par une coquille intérieure dont l'orientation est opposée à celle de la coquille extérieure relativement à la matière.

Pour traiter plusieurs coquilles :

1. calculer les intersections entre coquilles candidates ;
2. assembler les composantes qui se touchent ;
3. classifier les coquilles qui ne touchent rien par contenance ;
4. reconstruire la forêt ou la hiérarchie de contenance ;
5. déduire quelles composantes sont matière ou vide.

### Direction proposée pour ngk

Ne pas commencer par le cas général complet. Construire par paliers :

1. deux solides manifold, une coquille extérieure chacun ;
2. ajout des solides disjoints et de la contenance complète ;
3. ajout des cavités ;
4. ajout des résultats à plusieurs composantes ;
5. extension non-manifold si elle est réellement requise.

L'architecture des données doit toutefois éviter de rendre les étapes 2 à 4 impossibles.

---

## 16. Robustesse : règles de conception

### 16.1 Une question géométrique, une décision

Si plusieurs cellules dépendent de la même intersection, elles doivent référencer la même décision canonique.

### 16.2 Séparer mesure et décision

Une routine peut produire :

```text
valeur estimée + borne d'erreur + conditionnement
```

Une couche de décision transforme ensuite cela en :

```text
positive | négative | nulle prouvée | incertaine
```

### 16.3 Ne pas utiliser une seule tolérance universelle

Les distances 3D, paramètres de courbe, paramètres UV, angles et tailles de boîte ne vivent pas dans le même espace et ne doivent pas nécessairement partager le même seuil.

### 16.4 Canoniser avant de modifier

Avant le découpage :

- regrouper les événements supposés identiques ;
- vérifier leur compatibilité topologique ;
- choisir des paramètres canoniques ;
- détecter les contradictions.

### 16.5 Échouer proprement sur l'incertain

Un résultat `Uncertain` ou `DegenerateUnsupported` est préférable à une carte valide au sens des involutions mais représentant une mauvaise frontière.

### 16.6 Valider plusieurs couches

À la fin de la transaction :

1. axiomes GMap ;
2. index et attributs ;
3. cycles et pcurves de faces ;
4. fermeture des coquilles ;
5. orientation ;
6. absence d'auto-intersection indésirable ;
7. cohérence géométrie-topologie.

---

## 17. Intersection de surfaces NURBS : contrat nécessaire au booléen

L'algorithme booléen ne devrait pas dépendre de la manière interne dont une courbe d'intersection est calculée. Il doit dépendre d'un contrat riche.

### Limite du résultat actuel

L'intersection surface/surface actuelle approxime les deux surfaces par des triangles, collecte des points puis les trie selon l'axe cartésien de plus grande étendue.

Cette approche est utile pour prototyper, mais ne peut pas garantir :

- l'ordre correct sur une boucle fermée ;
- la séparation de plusieurs branches ;
- la continuité dans les deux espaces UV ;
- le passage d'une couture périodique ;
- la détection des tangences et singularités ;
- une erreur géométrique bornée entre les échantillons.

### Contrat cible

```rust
struct SurfaceIntersectionBranch {
    curve_3d: Curve,
    pcurve_a: Curve2,
    pcurve_b: Curve2,
    samples: Vec<IntersectionSample>,
    topology: BranchTopology,
    quality: IntersectionQuality,
}

struct IntersectionSample {
    point: Point3,
    uv_a: Point2,
    uv_b: Point2,
}
```

Le chapitre 6 suggère une direction naturelle pour deux surfaces paramétriques : suivre la courbe dans l'espace `(u1, v1, u2, v2)` défini par `S1(u1,v1) = S2(u2,v2)`. Les deux pcurves sont alors produites simultanément au lieu d'être reconstruites après coup.

---

## 18. Correspondance avec le code actuel

### Ce qui existe déjà et va dans la bonne direction

- `BooleanOperand` efface la dimension de l'opérande tout en gardant un handle typé dans chaque variante.
- `BooleanContact` distingue points, courbes, recouvrements et régions.
- Une courbe de contact contient déjà une courbe 3D et deux pcurves.
- `BooleanIntersectionPlan` sépare le calcul non-mutant de l'application.
- `apply_boolean_splits` utilise une transaction.
- `BooleanLineage` conserve la relation entre cellules sources et fragments.
- Le plan est revalidé avant application.
- L'import d'un outil externe et le découpage peuvent être atomiques.
- Les tests couvrent déjà plusieurs contacts et le rollback.

### Ce qui reste conceptuellement après la préparation

Le code public se termine actuellement à la préparation et au découpage. Pour obtenir un booléen complet, il reste principalement :

1. une représentation canonique et orientée du graphe d'intersection ;
2. l'analyse systématique des voisinages ;
3. la classification des fragments ;
4. les règles pour les régions coïncidentes ;
5. la sélection union/intersection/différence ;
6. l'assemblage et la couture des fragments retenus ;
7. la reconstruction multi-coquilles ;
8. la validation géométrique du résultat ;
9. un résultat public riche et une politique de payload.

Cette liste est une observation sur l'état actuel du code, pas une affirmation du livre.

---

## 19. Proposition de découpage en modules

Les noms sont indicatifs.

```text
builders/boolean/
    mod.rs
    operand.rs          résolution et import des opérandes
    broad_phase.rs      paires de faces candidates
    contacts.rs         narrow phase et contacts canoniques
    graph.rs            graphe d'intersection
    imprint.rs          plan et application des subdivisions
    neighborhood.rs     analyse locale des secteurs
    classify.rs         classification et propagation
    select.rs           règles union/intersection/différence
    assemble.rs         réconciliation, couture et coquilles
    result.rs           lineage, diagnostics et résultat public
    errors.rs
```

Il ne faut pas créer ces fichiers mécaniquement dès maintenant. Le découpage devient utile lorsque les responsabilités existent réellement. L'idée importante est de conserver les frontières entre phases.

---

## 20. Plan d'implémentation progressif

### Étape A - Stabiliser la préparation

- contacts canoniques ;
- points d'arêtes globalement ordonnés ;
- imprints UV formant des chaînes et boucles valides ;
- lineage complète ;
- diagnostics sur les ambiguïtés.

### Étape B - Intersection de deux blocs transversaux

- deux solides manifold à une coquille ;
- faces planes ;
- aucun contact coïncident ;
- classification locale et propagation ;
- assemblage d'une coquille fermée.

Cette étape prouve l'architecture topologique sans attendre l'intersection NURBS générale.

### Étape C - Union et différence

- réutiliser le même plan de contact et les mêmes classifications ;
- changer uniquement la politique de sélection et d'orientation ;
- produire une lineage et des résultats homogènes entre opérations.

### Étape D - Cas sans courbe d'intersection

- solides disjoints ;
- `A` contenu dans `B` ;
- `B` contenu dans `A` ;
- solides identiques ;
- contact ponctuel ou tangent éliminé par régularisation.

### Étape E - Contacts dégénérés polyédriques

- sommet/face ;
- sommet/arête ;
- sommet/sommet ;
- arêtes collinéaires en recouvrement ;
- faces coplanaires de même orientation ;
- faces coplanaires d'orientation opposée.

### Étape F - Courbes et surfaces NURBS

- branches d'intersection ordonnées ;
- pcurves synchronisées ;
- boucles fermées ;
- trims périodiques ;
- tangences ;
- stratégie explicite pour les singularités non supportées.

### Étape G - Multi-coquilles

- cavités ;
- plusieurs composantes de résultat ;
- forêt de contenance ;
- orientation des coquilles intérieures.

---

## 21. Matrice minimale de tests

### Position relative sans intersection transversale

- deux blocs disjoints ;
- blocs identiques ;
- `A` strictement dans `B` ;
- `B` strictement dans `A` ;
- contact sommet/sommet seulement ;
- contact arête/arête seulement ;
- contact face/face tangent seulement.

### Intersections transversales

- blocs qui se chevauchent ;
- coupe traversant exactement une face puis plusieurs faces ;
- intersection produisant une boucle fermée ;
- intersection produisant plusieurs composantes.

### Coïncidences

- faces coplanaires avec recouvrement partiel ;
- faces coplanaires identiques, même orientation ;
- faces coplanaires identiques, orientations opposées ;
- arêtes collinéaires avec recouvrement partiel ;
- sommets presque coïncidents de façon non transitive.

### Topologie du résultat

Pour chaque opération :

- `validate_gmap` réussit ;
- les coquilles attendues sont fermées ;
- les normales sont correctement orientées ;
- aucune face ou arête orpheline inattendue ;
- chaque couture identifie des subdivisions isomorphes ;
- la lineage ne référence aucune clé supprimée ;
- un échec restaure exactement la carte initiale.

### Invariants algébriques utiles

Quand les entrées sont valides :

- `A ∪ B` doit être équivalent à `B ∪ A` ;
- `A ∩ B` doit être équivalent à `B ∩ A` ;
- `A - A` doit être vide ;
- `A ∩ A` doit être équivalent à `A` ;
- `A ∪ A` doit être équivalent à `A` ;
- appliquer deux fois un plan obsolète doit échouer atomiquement.

L'équivalence doit être testée géométriquement et topologiquement, pas par égalité des clés.

---

## 22. Questions d'API à décider pour ngk

Ces décisions méritent d'être tranchées avant que l'assemblage final soit profondément implémenté.

1. Le booléen produit-il toujours de nouveaux `SolidKey`, ou peut-il préserver un opérande lorsque rien ne change ?
2. Une opération modifie-t-elle les deux solides sources, les consomme-t-elle, ou construit-elle un résultat séparé dans le même `Model` ?
3. Le mode preview doit-il exposer le graphe de contact et les classifications sans mutation ?
4. Quelle politique décide de l'identité survivante lors d'une fusion de cellules ?
5. Comment sont fusionnés ou copiés les payloads ?
6. Le MVP refuse-t-il explicitement les résultats non-manifold ?
7. Une intersection peut-elle retourner plusieurs solides ?
8. Comment représente-t-on un résultat vide ?
9. Jusqu'où le résultat public expose-t-il la lineage et les diagnostics ?
10. Quelle garantie donne-t-on quand le calcul numérique est incertain ?

### Direction que je recommande

- L'opération travaille dans un `Model` transactionnel.
- Elle consomme logiquement les opérandes sélectionnés, sauf option contraire explicite.
- Elle peut produire zéro, un ou plusieurs `SolidKey`.
- Elle retourne une structure de résultat riche.
- Elle expose un mode de préparation/preview non mutant.
- Le MVP annonce clairement « manifold fermé ».
- Les cas numériques non résolus produisent une erreur structurée avec diagnostic.

---

## 23. Résumé opérationnel

Si une seule page de ce document doit être retenue, retenir ceci :

1. Un booléen B-rep transforme une frontière, pas seulement un ensemble de points.
2. La même intersection doit être partagée par les deux opérandes et toutes les faces adjacentes.
3. Les contacts forment un graphe temporaire qui peut être incomplet pendant sa construction.
4. Les arêtes sont découpées après regroupement global de leurs paramètres.
5. Les faces sont découpées après construction complète de leurs graphes UV.
6. L'analyse locale classe les fragments adjacents aux contacts.
7. La classification est propagée aux faces qui ne touchent aucune intersection.
8. Union, intersection et différence partagent le même pipeline ; leur politique de sélection change.
9. Les fragments retenus doivent être réconciliés, orientés, cousus et regroupés en coquilles.
10. Une couture GMap nécessite des cellules isomorphes et une politique de plongement.
11. La transaction ne doit être validée qu'après contrôle topologique et géométrique.
12. Une tolérance seule ne garantit jamais la cohérence des décisions topologiques.

---

## 24. Références de lecture

### Hoffmann

- Chapitre 2, §2.2.2 : opérations booléennes régularisées.
- Chapitre 2, §2.3 : représentation de frontière.
- Chapitre 2, §2.4 : validité topologique des solides B-rep.
- Chapitre 3, pp. 67-108 : opérations booléennes sur B-rep.
- Chapitre 4, pp. 111-151 : robustesse des opérations géométriques.
- Chapitre 5, §5.7, pp. 193-203 : identification des arêtes courbes.
- Chapitre 6, pp. 205-254 : intersections de surfaces.

### Référence GMap du projet

- définition des cellules comme orbites ;
- axiomes des involutions ;
- opérations `i-sew` et `i-unsew` ;
- test de compatibilité avant couture ;
- plongement des cellules ;
- mise à jour des plongements lors des fusions et séparations.

### Documentation ngk

- `docs/topology_orientation_refactor.md` ;
- `docs/model_api.md` ;
- `src/builders/boolean.rs` et `tests/builders/boolean.rs` pour l'état expérimental actuel.
