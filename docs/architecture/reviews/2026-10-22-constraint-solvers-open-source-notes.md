# Solveurs de contraintes : FreeCAD PlaneGCS et SolveSpace

> **Statut : étude de référence exploratoire, 22 octobre 2026.** Lecture de documentation officielle et de code source. Ce document décrit deux architectures existantes ; il ne constitue ni une décision pour NGK ni un classement des solveurs.

## Périmètre et versions

- **FreeCAD / PlaneGCS** : branche `main`, commit [`d3f4da791daa9e94f340b81770d13f090ec679c6`](https://github.com/FreeCAD/FreeCAD/commit/d3f4da791daa9e94f340b81770d13f090ec679c6), consulté le 22 octobre 2026.
- **SolveSpace / libslvs** : branche `master`, commit [`30e1c3ac0beb77fe3e098315b1bb662840ddc99a`](https://github.com/solvespace/solvespace/commit/30e1c3ac0beb77fe3e098315b1bb662840ddc99a), consulté le 22 octobre 2026.
- Les programmes n'ont pas été compilés ni exécutés pour cette étude. Les comportements décrits sont **observés dans le code** ou **documentés**, pas mesurés sur un corpus.

Ces deux exemples ne couvrent pas tout l'état de l'art. Ils rendent cependant observable la frontière entre l'application, la géométrie et le calcul numérique, y compris pour des courbes plus riches que des segments.

## 1. La distinction utile : trois représentations qui coopèrent

Le code de FreeCAD et l'interface de SolveSpace font apparaître trois rôles :

| Rôle | Exemple |
| --- | --- |
| Géométrie manipulée par l'application | Un segment ou un cercle dessiné, avec une identité que les contraintes peuvent référencer |
| Géométrie paramétrique de résolution | Un segment référant deux points ; un cercle référant son centre et son rayon |
| Inconnues numériques | Les coordonnées des points, les rayons et éventuellement des poids de spline |

**Synthèse de lecture :** accepter un cercle du noyau géométrique à l'entrée n'oblige pas à utiliser cet objet directement comme stockage numérique pendant la résolution. FreeCAD réalise justement les deux étapes : il accepte la géométrie hôte et construit une représentation adaptée au solveur. [Adaptation FreeCAD](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L913-L952), [représentations PlaneGCS](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/Geo.h#L40-L237), [modèle SolveSpace](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/exposed/DOC.txt#L5-L29).

Cette séparation est un constat sur ces implémentations. Elle n'impose pas une crate indépendante, une duplication de toute la bibliothèque de courbes, ni une API NGK particulière.

## 2. FreeCAD : objets géométriques hôtes, paramètres de résolution propres

### Entrée et sortie

**Observé :** `Sketch::addGeometry` accepte des `Part::Geometry` : segments, cercles, coniques et B-splines. L'ajout d'un segment ou cercle clone l'objet, extrait ses valeurs dans des `double` alloués, puis crée les objets GCS correspondants. Le cercle fournit trois inconnues : centre X/Y et rayon. Après succès numérique, `applySolution` et `updateGeometry` répercutent les valeurs dans les géométries ; une mise à jour invalide entraîne un retour aux valeurs précédentes. [Ajout et extraction](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L766-L819), [segment](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L913-L952), [cercle](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L1728-L1765), [application et validation](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L4993-L5038).

La géométrie hôte peut elle-même envelopper OCCT : `Part::GeomBSplineCurve` accepte un `Handle(Geom_BSplineCurve)` et expose pôles, poids, nœuds, multiplicités et degré. [Type hôte](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Part/App/Geometry.h#L315-L390).

### Géométrie GCS

**Observé :**

- `GCS::Point` contient deux pointeurs `double*`.
- `GCS::Line` contient deux points.
- `GCS::Circle` contient un point central et un pointeur vers le rayon.
- `GCS::Arc` ajoute angles et points de début/fin.
- `GCS::Curve` fournit notamment évaluation, normale et accès aux paramètres pour les contraintes. `DeriVector2` transporte valeurs et dérivées.

Il ne s'agit donc ni d'un ensemble de points sans notion de courbe, ni d'une résolution directement sur une BRep. La représentation de courbe est spécialisée pour l'évaluation et les dérivées dont les contraintes ont besoin. [Types et contrats GCS](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/Geo.h#L40-L255).

### B-splines : une prise en charge paramétrique, avec des limites précises

**Observé :** GCS possède bien un type `BSpline` comprenant pôles, poids, nœuds, degré et périodicité. `Geo.cpp` contient sa propre évaluation rationnelle et des calculs différentiels. Le degré et les multiplicités sont explicitement des données non résolues. [Structure](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/Geo.h#L358-L413), [évaluation](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/Geo.cpp#L870-L965).

Dans l'intégration Sketch, coordonnées des pôles et poids sont enregistrés comme paramètres variables lorsque la courbe n'est pas fixe. Les nœuds ne sont pas ajoutés à cette liste ; la sortie réécrit pôles et poids. Le chemin B-spline active aussi une nouvelle résolution après actualisation géométrique en raison de calculs OCCT. **La présence d'un type B-spline ne signifie donc pas que tous ses paramètres et toutes ses contraintes soient libres et uniformément pris en charge.** [Paramètres et nœuds](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L1527-L1669), [sortie](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L4901-L4936), [nouvelle résolution](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/Sketch.cpp#L809-L816).

### État, diagnostics et déplacement

**Observé :** `GCS::System` conserve contraintes, paramètres et décomposition en sous-systèmes. Son interface distingue initialisation, résolution, application et annulation. Elle expose nombre de degrés de liberté, contraintes conflictuelles, redondantes ou partiellement redondantes, et paramètres dépendants. BFGS, Levenberg–Marquardt et DogLeg sont disponibles. [État et algorithmes](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/GCS.h#L56-L139), [cycle et diagnostics](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/GCS.h#L609-L675).

Les contraintes temporaires de déplacement portent un tag particulier et une priorité plus faible que les contraintes permanentes. Le commentaire du contrat donne le cas d'un cercle : déplacer son bord favorise une variation de rayon avant un déplacement du centre quand les autres contraintes le permettent. Le déplacement a donc une sémantique algorithmique, même sans GUI dans le moteur. [Contrat des contraintes temporaires](https://github.com/FreeCAD/FreeCAD/blob/d3f4da791daa9e94f340b81770d13f090ec679c6/src/Mod/Sketcher/App/planegcs/GCS.h#L88-L105).

## 3. SolveSpace / libslvs : paramètres, entités et contraintes explicites

### Interface de bibliothèque

**Documenté et observé :** l'interface C expose ses propres `Slvs_Param`, `Slvs_Entity`, `Slvs_Constraint`, référencés par des handles entiers. Un point 2D réfère deux paramètres et un plan ; un segment réfère deux points ; un cercle réfère centre, normale et une entité distance portant son rayon. Aucune courbe OCCT ou autre objet hôte n'est nécessaire dans ce contrat. [Documentation des entités](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/exposed/DOC.txt#L74-L247), [structures publiques](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/include/slvs.h#L31-L136).

La courbe libre exposée par cette API est `SLVS_E_CUBIC`, une **Bézier cubique non rationnelle à quatre points**. Ce n'est pas une B-spline générale à degré, nœuds et poids arbitraires. Cette limite concerne l'API libslvs examinée ; elle ne décrit pas toutes les représentations du noyau de modélisation SolveSpace. [Définition documentée](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/exposed/DOC.txt#L191-L204), [catalogue public](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/include/slvs.h#L50-L67).

### Références par identifiant et organisation des tableaux

**Observé :** `Slvs_System` reçoit des tableaux séparés de paramètres, de géométries et de contraintes. Une contrainte réfère ses points et autres géométries par handles ; le catalogue inclut la coïncidence entre points distincts. Cela fournit une référence concrète pour une entrée par lots avec des identifiants propres au sketch. [Tableaux et références](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/include/slvs.h#L70-L156).

**Distinction de conception :** un pool avec identifiants définit comment les objets se référencent. Un stockage « structure of arrays » définit comment leurs champs sont rangés en mémoire. L'un n'impose pas l'autre : ici, les tableaux contiennent des structs, avec notamment des tableaux de références dans chaque struct. Le contrat observé ne constitue pas un SoA complet par type de primitive. Les pointeurs de paramètres GCS représentent encore un autre mécanisme de référence ; leur présence ne suffit pas non plus à conclure à un layout SoA.

### Deux formes d'API et d'état

**Observé :**

| API actuelle | Où reste l'état ? |
| --- | --- |
| `Slvs_Solve(Slvs_System*, group)` | Les tableaux fournis par l'appelant contiennent les valeurs initiales et reçoivent les nouvelles valeurs. L'état interne est reconstruit et nettoyé à chaque appel. |
| `Slvs_Add…`, `Slvs_SetParamValue`, `Slvs_SolveSketch` | Le sketch est conservé par la bibliothèque entre les appels, jusqu'à `Slvs_ClearSketch`. L'implémentation examinée utilise un état global interne, pas un objet contexte fourni par l'appelant. |

[Cycle tableau](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/src/slvs/lib.cpp#L988-L1122), [cycle conservé](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/src/slvs/lib.cpp#L839-L985), [déclarations publiques](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/include/slvs.h#L435-L510).

Les paramètres marqués comme déplacés sont favorisés : le solveur essaie de les changer moins, sans les verrouiller. Le verrouillage `WHERE_DRAGGED` est une contrainte distincte, qui peut surcontraindre. Les résultats exposent statut, degrés de liberté et, sur demande, contraintes problématiques. [Contrat drag et diagnostics](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/include/slvs.h#L139-L204).

Le site officiel précise que le solveur reste fortement lié au reste du programme, malgré l'interface bibliothèque. **« Utilisable séparément » ne signifie pas « conçu comme une bibliothèque sans couplage interne ».** [SolveSpace comme bibliothèque](https://solvespace.com/library.pl).

## 4. Ce que fait la résolution, expliqué sans imposer un algorithme à NGK

**Illustration mathématique :** un segment AB défini par quatre coordonnées peut avoir :

- horizontalité : `by - ay = 0` ;
- longueur L : `(bx - ax)² + (by - ay)² - L² = 0`.

Le moteur ajuste les inconnues pour ramener ces erreurs vers zéro. Une seule contrainte utilisateur peut produire plusieurs équations ; inversement, une représentation peut nécessiter des équations internes de validité. Dans libslvs, un arc défini par centre/début/fin impose implicitement l'égalité des deux rayons. [Exemple réel d'équation interne](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/exposed/DOC.txt#L222-L247).

**Documenté pour SolveSpace :** les contraintes deviennent des équations symboliques généralement résolues numériquement par une méthode de Newton modifiée, avec des cas particuliers par substitution. Pour les systèmes sous-contraints, le choix de correction exploite une résolution aux moindres carrés. La configuration initiale influence la solution choisie lorsqu'il en existe plusieurs. [Description officielle de la méthode](https://solvespace.com/tech.pl). Le code courant forme le Jacobien, calcule une correction et pondère les paramètres déplacés dans cette résolution. [Implémentation de la correction](https://github.com/solvespace/solvespace/blob/30e1c3ac0beb77fe3e098315b1bb662840ddc99a/src/system.cpp#L273-L409).

**Inférence pratique, sans garantie de continuité :** repartir des dernières coordonnées réussies donne un point de départ proche pour l'édition suivante. Cela peut vivre dans une struct ou dans des tableaux détenus par l'appelant. Cette réutilisation ne nécessite pas un historique complet des anciens sketches. Elle ne garantit pas d'éviter tous les sauts : une singularité ou la disparition d'une branche de solutions reste un problème numérique/géométrique.

## 5. Ce que cette comparaison établit, et ce qu'elle ne décide pas

- On peut garder une API familière acceptant la géométrie du noyau tout en utilisant des paramètres internes spécialisés : c'est le chemin FreeCAD observé.
- On peut exposer directement un petit modèle paramétrique de résolution, avec primitives et références stables : c'est le contrat libslvs observé.
- Les deux approches savent désigner un centre de cercle sans l'assimiler à un sommet d'une BRep.
- Un composant de résolution peut conserver son état ; une interface par appel peut également conserver la dernière solution chez l'appelant.
- Le catalogue des courbes, leurs paramètres variables et le catalogue des contraintes sont trois couvertures différentes. « Supporte les splines » seul est insuffisant.
- Ni cette lecture ni la présence d'un algorithme particulier ne prouvent une robustesse universelle, un ordre de performance ou la meilleure architecture pour NGK.

Les limites d'API et de paramètres ci-dessus sont vérifiées dans les versions indiquées. Le choix du module NGK, de ses types publics, de ses diagnostics et de son état reste ouvert à la discussion.

