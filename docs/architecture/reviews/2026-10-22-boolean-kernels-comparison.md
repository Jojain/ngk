# Booléens : NGK, OCCT et SMLib

> **Statut : étude de référence, 22 octobre 2026.** Comparaison du code et des contrats des trois noyaux, complétée par des tests NGK exécutés localement. Les recommandations sont des propositions, pas des décisions d’architecture adoptées.

## 1. Ce qu’il faut retenir

**NGK possède déjà la bonne décomposition générale d’un moteur booléen. Son retard le plus visible concerne la couverture des cas difficiles, la cohérence des décisions numériques et le domaine des résultats acceptés.** Remplacer la GMap ou réécrire toute la chaîne n’est pas la priorité qui ressort de cette étude.

Les trois noyaux calculent des contacts, découpent des frontières, décident quels fragments appartiennent au résultat, puis reconstruisent une frontière. Leur différence se joue dans les contrats entre ces étapes : comment plusieurs observations deviennent un même événement, comment une approximation reste compatible avec les deux surfaces, et comment on tranche une coïncidence ou une tangence.

| Question | Conclusion de l’étude |
| --- | --- |
| Quelle référence prendre pour élargir les booléens de NGK ? | **OCCT**, pour la richesse de ses opérations, ses résultats multiples, sa partition réutilisable, son infrastructure de consolidation et ses diagnostics. Cela établit une avance fonctionnelle, pas un taux de réussite universel. |
| Que regarder particulièrement dans SMLib ? | Le passage par une proto-topologie commune et la classification des secteurs autour des contacts. Son code est une deuxième référence de conception, avec ses propres choix de tolérances et de représentation. |
| Quels acquis conserver dans NGK ? | La transaction, le réseau d’intersections partagé, la filiation, les refus explicites lorsque la couverture est incomplète, et le chemin général NURBS accompagné de spécialisations analytiques. |
| Qui est le plus rapide ? | **Non établi.** Aucun corpus identique n’a été exécuté sur les trois moteurs. La présence de parallélisme ou d’une marque NVIDIA ne répond pas à cette question. |
| Que faire en premier ? | Rétablir le fuzzing, mesurer les résultats faux séparément des refus, corriger la propagation des budgets numériques, puis traiter coïncidences/tangences/périodiques. Étendre le résultat à zéro ou plusieurs solides donne aussi un gain fonctionnel direct. |

Les justifications, sources et limites de ces conclusions sont développées ci-dessous. Les deux [notes OCCT](2026-10-22-boolean-comparison-occt-notes.md) et [SMLib](2026-10-22-boolean-comparison-smlib-notes.md) conservent les détails de recherche et des références supplémentaires.

## 2. Périmètre et niveau de preuve

### Versions examinées

| Noyau | Référence |
| --- | --- |
| NGK | `0.0.12`, branche locale `dev`, commit `633672da62372dd40cac8a99839a03fe743cb7f2`. Arbre de travail propre avant la recherche. |
| OCCT | `8.0.1`, tag `V8.0.1`, commit `b8f597c677811d1f9f4d8a97f5ae2825c0353a42`, release publiée le 30 juillet 2026. [Release officielle](https://github.com/Open-Cascade-SAS/OCCT/releases/tag/V8.0.1). |
| SMLib | Noyau `10.0.0` dans **USD BRep `1.0.0`**, release publique du 9 octobre 2026 ; dépôt NVIDIA examiné au commit `c9c5979f04cea8c38213004d4f04df0dcfb40d05`. Le numéro du paquet et celui du noyau désignent deux choses différentes. [Release officielle](https://github.com/NVIDIA-Omniverse/usd-brep/releases/tag/v1.0.0), [version du noyau](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/inc/SmConfig.h#L28-L33). |

Le rapport distingue quatre niveaux :

1. **Observé dans le code** : un chemin est réellement implémenté dans la référence indiquée.
2. **Documenté** : le projet annonce un contrat ou une limitation ; cela ne prouve pas sa réussite sur toutes les entrées.
3. **Exécuté ici** : résultat des commandes NGK de la section 9.
4. **Signalé publiquement / inféré** : défaut rapporté par un utilisateur ou risque déduit du code, sans reproduction locale.

Les booléens OCCT et SMLib n’ont pas été exécutés dans cette étude. Aucun pourcentage comparatif de robustesse ni classement de vitesse n’est donc présenté. Une capacité observée dans un solveur surface/surface n’est pas automatiquement une capacité vérifiée du booléen solide complet.

### Vocabulaire

Dans NGK, sommet, arête, face et solide désignent les entités logiques. Les cellules de la GMap sont nommées explicitement `N-cellules`. La GMap encode la combinatoire et les incidences ; les supports géométriques et les pcurves donnent leur réalisation dans l’espace. Cette distinction est essentielle : vérifier les involutions ne vérifie pas que l’union contient effectivement ses deux opérandes. [Vocabulaire du projet](../../../CONTEXT.md), [validation NGK](../../../src/topology/validation.rs), lignes 88–190.

## 3. Pourquoi un booléen est difficile

```mermaid
flowchart LR
    A[Valider les opérandes] --> B[Détecter les contacts]
    B --> C[Unifier les événements et sections]
    C --> D[Découper les frontières]
    D --> E[Classer les fragments]
    E --> F[Sélectionner et assembler]
    F --> G[Vérifier géométrie et topologie]
```

Pour `A − B`, il faut conserver les morceaux de la frontière de A extérieurs à B, ajouter les morceaux utiles de B avec leur orientation retournée, puis les raccorder. Calculer une belle courbe d’intersection ne suffit pas : elle doit découper les deux opérandes en fragments compatibles.

Les configurations transverses sont les plus simples : les surfaces se croisent avec des normales suffisamment différentes. Les difficultés se concentrent dans les changements de dimension et les cas mal conditionnés :

- deux faces presque confondues ou partiellement coïncidentes ;
- une tangence qui produit un point, une courbe ou une région commune ;
- une boucle fermée entièrement intérieure aux domaines des surfaces ;
- une section traversant une couture périodique ou un pôle ;
- un détail beaucoup plus petit que l’ensemble des opérandes ;
- plusieurs surfaces qui découvrent séparément ce qui devrait être le même sommet.

**Trois propriétés doivent être évaluées séparément** :

| Propriété | Exemple d’échec |
| --- | --- |
| Validité combinatoire | Une frontière reste ouverte ou des incidences se contredisent. |
| Cohérence géométrique | Une courbe 3D et sa pcurve évaluée sur la surface ne se superposent pas dans le budget annoncé. |
| Justesse ensembliste | Le solide est fermé et orienté, mais l’union a perdu une partie d’un opérande. |

Cette séparation explique pourquoi un moteur peut réussir un contrôle « manifold » et rendre un résultat faux. Elle explique aussi pourquoi un refus explicite et reproductible est préférable à un succès incorrect, tout en restant une lacune fonctionnelle à réduire.

## 4. NGK : ce qui existe réellement

### 4.1 Une chaîne complète, avec une décision finale commune

Le point d’entrée valide les solides, résout un budget de tolérances, puis ouvre une transaction. `arrangement::arrange` intersecte et découpe chaque paire d’opérandes pertinente, y compris outil contre outil. Ensuite viennent une classification, une sélection et un assemblage du résultat final. Ce n’est donc pas une simple succession de booléens binaires dont chaque résultat intermédiaire devrait être connecté. [Orchestration](../../../src/builders/boolean/operations.rs), lignes 63–125 ; [arrangement](../../../src/builders/boolean/arrangement.rs), lignes 1–123.

Les sémantiques publiques sont :

- union : `target ∪ t₁ ∪ … ∪ tₙ` ;
- différence : `target − (t₁ ∪ … ∪ tₙ)` ;
- intersection : `target ∩ t₁ ∩ … ∩ tₙ`.

Les outils répétés sont dédupliqués. Les tests couvrent notamment des outils qui ne deviennent connectés que grâce à un outil ultérieur, les recouvrements outil/outil et certaines préservations de clés. [API de modélisation](../../../src/modeling/solids.rs), lignes 161–197 ; [tests multi-outils](../../../tests/builders/boolean_tools.rs).

### 4.2 Des fondations géométriques déjà sérieuses

NGK dispose d’un BVH déterministe pour filtrer les couples de faces, de bornes conservatrices et de caches de géométries préparées/domaines rognés pendant le calcul des contacts. Un problème de vitesse ne peut donc pas être résumé à « il manque un broad phase ». [Bornes et BVH](../../../src/builders/boolean/broad_phase.rs), notamment lignes 235–372 ; [caches](../../../src/builders/boolean/contacts.rs), lignes 31–115.

La résolution commence par les spécialisations analytiques reconnues ; elle utilise sinon le chemin NURBS/Bézier. La table surface/surface comprend plan/plan, plan/sphère, plan/cylindre et sphère/sphère ; cylindre/cylindre ne dispose pas de ce raccourci dans cette table. Les solveurs généraux emploient subdivision, correction numérique, suivi de branches et ajustement synchronisé des courbes. [Dispatch analytique](../../../src/geometry/dim3/intersections/analytic/surface_surface.rs), lignes 49–96 ; [solveur général](../../../src/geometry/dim3/intersections/surface_surface.rs), lignes 293–408.

La recherche des boucles intérieures existe déjà : arguments de signes Bernstein et monotonie lorsqu’un opérande est plan, subdivision et cônes de normales pour une paire générale. Si les budgets ne permettent pas de conclure, la couverture devient incomplète. Les tests exécutés incluent une boucle paraboloïde/plan et deux boucles intérieures entre cylindres. [Recherche d’amorces](../../../src/geometry/dim3/intersections/surface_surface/seeds.rs), lignes 194–338 et 750–845 ; [tests d’intersection](../../../tests/geometry/dim3/intersections.rs), lignes 340 et 959.

Le réseau commun porte événements, sections, incidences locales, orientations, pcurves et régions communes. Sa finalisation recoupe les sections aux événements compatibles jusqu’à un point fixe borné. Une section doit être réalisée des deux côtés avant l’assemblage. C’est un acquis architectural important. [Réseau](../../../src/builders/boolean/graph.rs), lignes 15–180, 598–620 et 904–993.

### 4.3 La classification couvre déjà des surfaces courbes

Le classificateur choisit entre plans, intersections analytiques rayon/surface et surfaces préparées NURBS. Il exige deux rayons acceptés concordants ; les rayons tangents, trop proches d’une frontière ou issus d’une recherche incomplète sont abandonnés. La position d’un témoin éventuellement proposé par la tessellation est réévaluée sur la surface. [Classification](../../../src/builders/boolean/classify.rs), lignes 24–145, 258–310 et 359–409.

**Le commentaire de `operations.rs` indiquant une classification limitée aux frontières polygonales planes est périmé.** Il contredit ces chemins et les tests solides avec cylindres et sphères. Les anciens documents datés de septembre décrivent également un état antérieur. Le rapport suit le code actuel.

Chaque fragment est classé indépendamment contre les autres opérandes, avec arrêt anticipé lorsque la sélection est déjà déterminée. Cette stratégie évite une propagation fondée sur un graphe de barrières incomplet ; elle peut coûter cher quand il y a beaucoup de fragments. C’est un compromis visible, pas un défaut de conception démontré. [Boucle de classification](../../../src/builders/boolean/classify.rs), lignes 453–500.

### 4.4 Les principales frontières actuelles

**Résultats restreints.** Le résultat solide public contient exactement un solide connecté. Le vide et plusieurs enveloppes extérieures provoquent respectivement `EmptyResult` et `DisconnectedResult`, avec annulation. Les cavités sont acceptées. Une union de deux pièces séparées ou une coupe séparant une pièce en deux sont donc refusées par contrat, même si la géométrie est parfaitement ordinaire. [Assemblage](../../../src/builders/boolean/assemble.rs), lignes 148–149 et 222–252 ; [test du contrat](../../../tests/builders/boolean.rs), lignes 667–705.

**Coïncidences courbes incomplètes.** Les recouvrements coplanaires ont un chemin dédié. En revanche, une coïncidence de supports courbes peut rester un `OverlapCandidate` avec `CoincidentRegionResolutionNotImplemented`. Le chemin général vérifiant l’identité de deux NURBS exige une représentation compatible ; il ne constitue pas une reconnaissance générale du même support sous reparamétrage. L’arrangement refuse les recouvrements non résolus. Certaines tangences analytiques et plane/patch sont traitées ; la tangence générale et les singularités restent des familles difficiles. [Contacts](../../../src/builders/boolean/contacts.rs), lignes 733–905 ; [coïncidences](../../../src/geometry/dim3/intersections/surface_surface.rs), lignes 205–218, 325–338 et 532–549 ; [refus](../../../src/builders/boolean/arrangement.rs), lignes 105–113.

**Garanties numériques à préciser.** Pour les branches suivies puis ajustées, `IntersectionQuality::certified` est calculé à partir des résidus et de l’erreur maximale mesurée sur des points de vérification : états du suivi, milieux et 129 positions globales. Les branches analytiques suivent leur propre évaluation de fidélité. Le réseau vérifie aussi l’accord pcurve/courbe à neuf positions par section. Ces contrôles échantillonnés sont utiles ; ils n’établissent pas une borne continue de l’erreur entre les points évalués. La couverture de recherche et la qualité d’ajustement sont deux propriétés distinctes. [Qualité](../../../src/geometry/dim3/intersections.rs), lignes 264–310 ; [ajustement](../../../src/geometry/dim3/intersections/surface_surface/fitting.rs), lignes 149–171 et 718–755 ; [branches analytiques](../../../src/geometry/dim3/intersections/surface_surface.rs), lignes 220–278 ; [vérification du réseau](../../../src/builders/boolean/graph.rs), lignes 973–993.

**Tolérances déjà contextualisées, mais propagation perfectible.** Le budget est mis à l’échelle des opérandes, puis fixé pour l’opération. Deux observations méritent une vérification ciblée : `probe_margin`, dérivé d’une distance 3D, est comparé à une distance dans le domaine UV ; le rayon NURBS reconstruit ses options à partir des valeurs par défaut en ne recopiant que les tolérances linéaire et paramétrique. Ce sont des points précis à auditer, pas la preuve reproduite d’un résultat faux. [Budget](../../../src/builders/boolean/tolerance.rs), lignes 57–165 ; [rayon et témoin](../../../src/builders/boolean/classify.rs), lignes 274–278 et 387–404.

## 5. OCCT : une infrastructure booléenne plus large

### 5.1 Une partition réutilisable

OCCT sépare le calcul des interférences (`BOPAlgo_PaveFiller`) de la construction des fragments (`BOPAlgo_Builder`). General Fuse, opérations usuelles, Section, Splitter et Cells Builder exploitent cette infrastructure. `PerformWithFiller` permet de réutiliser des intersections déjà calculées ; Cells Builder permet de sélectionner différentes combinaisons de fragments. **NGK expose déjà une préparation binaire séparée**, avec calcul du réseau puis application des découpes. OCCT va plus loin avec une partition sélectionnable et la construction de plusieurs expressions sans recalculer les interférences. [Builder](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.hxx#L43-L70), [réutilisation](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.hxx#L147-L153), [Cells Builder](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CellsBuilder.hxx), [préparation publique NGK](../../../src/builders/boolean/operations.rs), lignes 129–160 et 410–425.

Son résultat est un `TopoDS_Shape`, capable de contenir plusieurs solides et des structures de différentes dimensions, sous les règles propres à chaque opérateur. `Section` exprime les intersections de frontières ; `Common` ne signifie pas systématiquement « rendre tous les contacts de toute dimension ». [Modèle de formes](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingData/TKBRep/TopoDS/TopoDS_Shape.hxx#L30-L40), [contrat booléen](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BOP.hxx#L30-L49).

### 5.2 Son apport essentiel : maintenir la cohérence des incidences

Le code 8.0.1 traite les couples sommet/sommet, sommet/arête, arête/arête, sommet/face, arête/face, puis face/face, avec des consolidations entre étapes. Après certaines augmentations de tolérances de sommets, il **relance des recherches d’intersections**. La nouvelle zone d’incertitude peut avoir créé une incidence qui n’existait pas au premier passage. [Pipeline réel et reprises](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.cxx#L234-L424).

Les fragments d’arêtes sont organisés avec sommets paramétrés, intervalles découpés et blocs communs. Les courbes 3D, pcurves, micro-arêtes et arêtes dégénérées font l’objet de traitements spécifiques avant la reconstruction. Les faces de même domaine sont consolidées. L’intérêt pour NGK est le **contrat de cohérence entre phases**, davantage que les noms de classes. [Structures de blocs](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PaveBlock.hxx), [blocs communs](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_CommonBlock.hxx), [construction des faces](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_2.cxx).

Les intersections utilisent des traitements analytiques et des lignes de marche ensuite approchées. `IntTools_FaceFace` suit aussi la tolérance effectivement atteinte entre courbes et surfaces. OCCT n’apporte donc pas une solution symboliquement exacte aux intersections générales de NURBS. [Résolution face/face](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx#L395-L548), [approximation](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx#L1171-L1269).

### 5.3 Des options utiles avec des préconditions

OCCT associe des tolérances aux entités et expose une tolérance supplémentaire `FuzzyValue`. Ce réglage peut résoudre des petits jeux et des quasi-coïncidences ; il peut aussi absorber des détails. `Glue` évite du travail lorsque l’appelant garantit certaines coïncidences : la promesse n’est pas vérifiée par le moteur. Le mode non destructif copie les sous-formes à modifier ; il ne fournit pas à lui seul la transaction complète de NGK. [Options du PaveFiller](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.hxx), [contrat Glue](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_GlueEnum.hxx), [détails documentés](2026-10-22-boolean-comparison-occt-notes.md#api-et-exploitation).

L’historique distingue `Modified`, `Generated` et `IsDeleted`. Il aide à suivre les sources d’un résultat, sans garantir des identités persistantes pour toute reconstruction. NGK possède déjà sa propre filiation et ses politiques de payload ; ce besoin n’est pas absent de NGK. [Historique OCCT](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderShape.hxx), [transactions NGK](../transactions.md).

OCCT dispose de parallélisme effectif dans les intersections et la construction, avec contextes propres aux threads, ainsi que d’options OBB et de caches. NGK exécute séquentiellement les boucles de contacts et d’arrangement examinées. Cela donne à OCCT davantage de moyens de passage à l’échelle ; le gain dépend du cas et doit être mesuré. [Parallélisme](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_Parallel.hxx), [face/face](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_6.cxx#L490-L529).

### 5.4 Des défauts encore concrets

Un blocage de section sur géométries cylindriques a été corrigé par une normalisation périodique plus robuste en juillet 2026 ; la correction est incluse dans 8.0.1. Le cas rapporté comprenait une entrée invalide. Il montre un besoin de terminaison robuste, pas un défaut encore présent dans le tag étudié. [Signalement #1288](https://github.com/Open-Cascade-SAS/OCCT/issues/1288), [correction #1329](https://github.com/Open-Cascade-SAS/OCCT/pull/1329).

Un autre signalement, encore ouvert à la consultation, concerne une pcurve de couture absente après une opération sur solides presque coïncidents. L’auteur rapporte le crash sur 7.9.3 et le risque sur master ; le tag 8.0.1 contient le déréférencement non protégé identifié. **Le crash n’a pas été reproduit ici.** [Issue #1456](https://github.com/Open-Cascade-SAS/OCCT/issues/1456), [code concerné](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.cxx#L150-L153).

Enfin, `ArgumentAnalyzer::TestTangent()` est vide dans le tag examiné malgré l’interface annoncée. Le moteur traite bien des tangences ailleurs ; cette méthode ne constitue pas une vérification préventive fonctionnelle. [Implémentation](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ArgumentAnalyzer.cxx#L676-L679).

## 6. SMLib : proto-topologie et classification locale

### 6.1 Le noyau derrière USD BRep

L’ouverture récente concerne le dépôt NVIDIA **USD BRep**, qui contient le noyau SMLib et ses couches d’accès. Le format USD, le paquet Python et l’algorithme booléen ne doivent pas être confondus. Les chemins examinés sont des implémentations C++ sur CPU ; du TBB existe dans certaines préparations d’arbres et de frontières, tandis que la boucle de paires de faces examinée est séquentielle. Aucune accélération GPU du booléen BRep n’est établie ici. [Préparations parallèles](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmTopologyIntersector.cpp#L1128-L1145), [frontières et boucle de faces](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmTopologyIntersector.cpp#L1657-L1744).

### 6.2 Comment il construit le résultat

Le chemin observé passe par `SmMerge` et `SmTopologyIntersector::IntersectInsertRelate`. Il sépare la détection des intersections, leur résolution, leur insertion dans la topologie et l’établissement des relations entre les opérandes. Des données de proto-topologie permettent de représenter et rapprocher les observations avant leur application à la BRep. Les régions partiellement coïncidentes ont une représentation dédiée, `ProtoFace`, plutôt qu’une section transversale artificielle. [Consolidation et insertion](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmTopologyIntersector.cpp#L1146-L1268), [contacts de faces](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmTopologyIntersector.cpp#L1794-L1834).

La représentation explicite les usages orientés des faces et arêtes ainsi que l’ordre radial autour des arêtes. La décision booléenne s’appuie sur les régions et les secteurs pleins/vides autour des contacts. Les faces restantes sont localisées dans les régions de l’autre opérande. Cette combinaison donne une autre façon de traiter les frontières communes et les configurations non manifold. **L’intérêt pour NGK est de rendre les relations locales de contact plus explicites**, tout en conservant un classificateur global pour les composantes qui en ont besoin. [Classification et reconstruction](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmMerge.cpp#L2464-L2508).

Les intersections disposent de spécialisations analytiques et d’un chemin général d’intersection de surfaces. Le dispatch d’un plan reconnaît notamment plan, cône/cylindre, sphère, extrusion et révolution avant le fallback général. Le chemin général suit puis ajuste des morceaux de courbes ; `TestSpanAccuracy` contrôle cinq points intérieurs et les tangentes. « BRep exacte » désigne ici une représentation par surfaces et courbes, par opposition à une sortie seulement polygonale. Cela n’équivaut pas à une preuve d’exactitude algébrique du booléen général. [Dispatch](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmPlane.cpp#L1970-L2066), [contrôle d’ajustement](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmSurfaceIntersector.cpp#L3484-L3608).

Le code distingue de nombreuses classes de tangence et de singularité. Certaines directions singulières sont recherchées à partir des courbures principales et des indicatrices de Dupin. C’est un traitement géométrique spécialisé à étudier pour NGK ; sa présence ne garantit pas la résolution de tous les contacts. [Classes de points](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/inc/SmSurfaceIntersector.h#L49-L113), [directions singulières](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmAdvSurfaceIntersector.cpp#L973-L1016).

### 6.3 Les nuances qui changent la comparaison

La configuration examinée active `SM_USE_OLDTOL`. Un modèle de tolérances plus récent existe sous conditions de compilation ; sa présence dans les sources ne suffit pas à l’attribuer à la configuration publiée. Le mode automatique de `SmMerge` tient notamment compte des tolérances des BReps et de la plus petite arête. Un recentrage temporaire limite aussi le bruit numérique loin de l’origine. Ces heuristiques apportent des réponses concrètes, dont les seuils doivent être évalués sur des géométries multiscalaires. [Configuration](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/inc/SmConfig.h#L224-L256), [choix des tolérances](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmMerge.cpp#L94-L157), [recentrage](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmMerge.cpp#L2330-L2384).

De même, `merge_breps` reçoit plusieurs entrées mais procède par opérations binaires successives. Une différence sur `[a, b, c]` signifie **`(c − b) − a`**, comme le confirme le test publié. Ce n’est pas la sémantique cible/outils de NGK. Pour toute opération, les approximations intermédiaires peuvent rendre l’ordre observable. [Évaluation de l’arbre](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmMerge.cpp#L1875-L1994), [test de différence](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SmPyLib/tests/test_omni_solid.py#L2197-L2215).

Le booléen natif standard modifie le premier BRep et consomme normalement le second ; les wrappers Python transfèrent la propriété des entrées. Des callbacks et un collecteur de changements permettent de suivre splits, merges et suppressions. Une annulation atomique de toutes les erreurs n’a pas été établie dans les chemins examinés. Certains échecs de résolution sont réduits à des avertissements avant poursuite ; ce choix doit être pris en compte dans la définition d’un « succès ». [Mutation des entrées](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmMerge.cpp#L2445-L2462), [suivi des changements](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/inc/SmTrackTopologyChanges.h#L34-L94), [résolution et avertissements](https://github.com/NVIDIA-Omniverse/usd-brep/blob/c9c5979f04cea8c38213004d4f04df0dcfb40d05/source/SMLib/src/SmTopologyIntersector.cpp#L2251-L2295).

SMLib est donc une référence intéressante sur la représentation des contacts et la classification locale. La supériorité de sa robustesse générale ou de ses performances sur OCCT n’est pas établie par cette lecture.

### 6.4 Des limites publiquement observables

Dans l’[issue #1](https://github.com/NVIDIA-Omniverse/usd-brep/issues/1), un utilisateur du wheel USD BRep 1.0.0 signale qu’une union avec une copie miroir perd un opérande, sans erreur, avec un résultat déclaré manifold. Une rotation donnant le même ensemble spatial réussit dans son exemple. Le signalement vise donc la combinaison représentation réfléchie/coïncidences rognées, pas toute union. La [PR #17](https://github.com/NVIDIA-Omniverse/usd-brep/pull/17), ouverte et non fusionnée lors de la recherche, propose de corriger un contrôle de planéité utilisant une boîte munie d’une marge de collision. Son auteur rapporte une reproduction sur le commit de base étudié puis une correction ; ces résultats n’ont pas été réexécutés ici.

L’[issue #16](https://github.com/NVIDIA-Omniverse/usd-brep/issues/16) signale des erreurs de relation/distance sur certaines configurations cylindriques et des échecs de booléen sur des placements tangents. La relation de distance est une autre opération : ses défauts ne doivent pas être automatiquement comptés comme des échecs du booléen.

Ces signalements suffisent à réfuter une confiance fondée seulement sur « noyau établi », « exact » ou « manifold ». Ils ne permettent pas de calculer un taux d’échec comparatif.

## 7. Comparaison par capacité

Ce tableau résume les chemins décrits et sourcés dans les sections 4 à 6. « Plus large » qualifie le domaine fonctionnel observé ; « non établi » indique l’absence de mesure commune.

| Axe | NGK | OCCT | SMLib | Conséquence pour NGK |
| --- | --- | --- | --- | --- |
| Représentation | GMap + entités logiques + géométrie | BRep hiérarchique, orientations et placements | BRep avec usages orientés, régions et ordre radial | La représentation topologique seule n’explique pas la robustesse. |
| Chaîne booléenne | Arrangement de toutes les paires, décision finale commune | Partition commune et opérateurs réutilisant les interférences | Intersections, proto-topologie, insertion et relations | Conserver les phases NGK, renforcer leurs contrats. |
| Sortie solide | Un solide connecté ; vide/multiples refusés | Sorties vides ou composées, domaine d’opérations plus large | Modèle de régions et BRep plus général | Étendre le contrat NGK à zéro, un ou plusieurs solides. |
| Multi-opérandes | Arrangement global, passes par paires séquentielles | General Fuse / groupes objets-outils / Cells Builder | `merge_breps` compose des binaires | NGK possède déjà un avantage structurel sur ce wrapper SMLib. |
| Surfaces courbes | Analytique ciblé + chemin NURBS/Bézier | Analytique, marche et approximation | Analytique + solveurs généraux et ajustements | Mesurer les trous de couverture du chemin général. |
| Boucles d’intersection intérieures | Recherche et diagnostic de couverture présents | Infrastructure générale d’intersection | Infrastructure générale d’intersection | Ni leur présence ni leur exhaustivité universelle ne se déduisent d’une démo. |
| Coïncidences | Planaires dédiées ; courbes générales incomplètes | Blocs communs et faces de même domaine | Relations de contacts et secteurs | Chantier majeur : support commun + recouvrement rogné. |
| Tangences | Plusieurs cas spécifiques ; refus possibles | Traitements dédiés, défauts encore signalés | Traitements dédiés, défauts encore signalés | Catégoriser les tangences par dimension et conditionnement. |
| Périodiques et singularités | Domaines dépliés, pcurves, tests courbes | Code spécialisé, coutures et dégénérées | Représentation et traitements spécialisés | Renforcer les cas paramétrés couture/pôle/reparamétrage. |
| Incertitude numérique | Budget fixé par opération, qualité et couverture explicites | Tolérances d’entités, fuzzy, consolidation avec reprises | Tolérances géométriques et de zone ; configuration legacy active | Séparer seuil de décision, erreur atteinte et politique de fusion. |
| Classification | Témoins et rayons, traitement des coïncidences | Appartenance et reconstruction à partir des interférences | Relations régionales et secteurs locaux | Ajouter des relations locales fiables, puis mesurer l’intérêt d’une propagation. |
| Nettoyage du résultat | Healing dans la transaction | Simplification de même domaine et historique | Traitements de merge/régions et options dédiées | Contrôler que le nettoyage préserve le même ensemble spatial. |
| Données applicatives | Clés, filiation, payload et transaction | Historique Modified/Generated/IsDeleted | Attributs et structures de relations | Tester les garanties utiles au modeleur, sans confondre filiation et nommage persistant. |
| Isolation d’un échec | Annulation du modèle sur erreur récupérable | Mode non destructif configurable ; transaction applicative distincte | API et modes de mutation à examiner selon l’entrée utilisée | Préserver la garantie forte de NGK et mesurer son coût. |
| Parallélisme | Boucles examinées séquentielles | Parallélisme explicite sur plusieurs phases | TBB partiel ; parallélisme des booléens complets non établi | Profiler puis paralléliser des observations indépendantes avec fusion déterministe. |
| Exactitude / vitesse globale | Non établie au-delà du corpus testé | Non garantie universellement | Non garantie universellement | Comparer les trois sur le même contrat et le même corpus. |

### Un piège important : « intersection multi-outils » ne veut pas toujours dire la même chose

NGK calcule `A ∩ B ∩ C`. Le Common OCCT entre objets `[A]` et outils `[B, C]` sélectionne, pour les domaines solides, `A ∩ (B ∪ C)`.

Si B et C sont disjoints et traversent chacun A, le résultat NGK est vide ; cet appel OCCT conserve deux morceaux. Ce désaccord est **sémantique**, pas un défaut de robustesse. Une comparaison doit adapter les appels ou sélectionner les cellules de la même expression. [Sélection OCCT](https://github.com/Open-Cascade-SAS/OCCT/blob/V8.0.1/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BOP.cxx#L621-L775), [contrat NGK](../../../src/builders/boolean/operations.rs), lignes 63–79.

## 8. Les cas difficiles à travailler, et ce que chaque référence apporte

| Famille | Difficulté réelle | Apport de la comparaison |
| --- | --- | --- |
| Faces courbes partiellement confondues | Reconnaître le même support, comparer les domaines rognés, garder une frontière une seule fois | OCCT : même domaine et blocs communs ; SMLib : relations de régions ; NGK : compléter la résolution des overlaps. |
| Tangence et quasi-tangence | Le rang du problème change ; un petit résidu peut cacher une grande erreur de position du contact | Les trois utilisent des traitements spécifiques ; les cas restants doivent être diagnostiqués par type de contact. |
| Coutures et pôles | Plusieurs coordonnées UV représentent le même point 3D ; certaines directions paramétriques se contractent | Partager une réalisation cohérente du domaine entre booléens, mesure, validation et tessellation. |
| Petits détails dans un grand assemblage | Une tolérance adaptée à l’étendue globale peut absorber un détail local | Inspirer des budgets locaux traçables, sans accepter une croissance illimitée de tolérance. |
| Copies miroir et reparamétrages | Même ensemble spatial, représentations et orientations différentes | Transformer ces équivalences en tests métamorphiques ; le signalement SMLib #1 montre leur intérêt. |
| Contacts triples et outils recouvrants | Plusieurs couples doivent produire le même événement et la même subdivision | Conserver le réseau canonique NGK ; vérifier cohérence et invariance à l’ordre des outils. |
| Coupes créant plusieurs morceaux | Le calcul peut être correct, mais le type de sortie ne peut pas le porter | Étendre l’API avant d’interpréter chaque refus comme une faiblesse du solveur. |
| Résultat fermé mais faux | Les vérifications combinatoires n’observent pas l’ensemble spatial attendu | Croiser appartenance, volume, résidus, composantes et identités ensemblistes. |

Le mémo existant [One boundary walk for face domains](../refactors/2026-10-15-face-domain-boundary-walk.md) décrit déjà des duplications entre domaine rogné, tessellation et mesure. Cette convergence est directement pertinente pour les booléens périodiques. Elle mérite une étude ciblée, pas un refactor général sans cas de validation.

## 9. Vérifications effectuées sur NGK

Commandes exécutées sur le commit indiqué, sans modification du code :

| Commande | Résultat | Portée |
| --- | --- | --- |
| `cargo test --release --test builders boolean -- --test-threads=1` | **111 réussis, 0 échec, 0 ignoré**, 166 filtrés ; exécution des tests : 1,99 s | Filtre textuel `boolean` : préparation, opérations solides/faces, réseau, budgets, outils et un test de suppression. |
| `cargo test --release --test geometry intersections -- --test-threads=1` | **36 réussis, 0 échec, 0 ignoré**, 230 filtrés ; exécution : 0,04 s | Intersections 3D, cas analytiques/généraux, couverture, budgets et ajustement. |
| `cargo test --release --test boolean_fuzz -- --cases 24 --seed 20261022 --no-shrink` | **Échec de compilation, quatre erreurs E0277 ; aucun cas exécuté** | `tests/fuzz/boolean.rs:382–385` utilise les anciens appels binaires ; l’API attend une collection de références d’outils. |

La première compilation signale aussi un import inutilisé dans `tests/builders/solids.rs:14`. Ces durées sont celles des groupes de tests, **pas des latences comparatives des moteurs**. Les tests utilisent les fonctionnalités par défaut ; ni les bindings Python/WASM ni la suite complète ne sont validés par ces commandes.

Deux nuances sont importantes :

- les tests de budget vérifient la terminaison et acceptent aussi une erreur typée ; leur succès ne prouve pas que chaque scène a produit un solide ;
- le fuzzing est explicitement exclu du lancement ordinaire de la suite. Sa présence dans le dépôt ne signifie donc pas qu’il fonctionne ou qu’il est régulièrement exécuté. [Tests de budget](../../../tests/builders/boolean_budget.rs), lignes 45–72 ; [cible de fuzzing](../../../Cargo.toml) et [appels obsolètes](../../../tests/fuzz/boolean.rs), lignes 374–386.

Les 147 tests ciblés réussis confirment que NGK dispose d’un socle fonctionnel réel. Ils ne mesurent pas son taux de réussite sur une population représentative de modèles CAD.

## 10. Priorités proposées pour NGK

### P0 — Rendre la robustesse mesurable

**Action.** Remettre le programme de fuzzing en accord avec l’API multi-outils et vérifier sa compilation dans la CI. Maintenir une campagne explicite avec graines, cas minimisés et budget d’exécution. Séparer les catégories : succès correct, vide attendu, résultat multiple, refus de couverture, ambiguïté, crash/panic, résultat annoncé correct mais faux.

Le fuzzing existant contient déjà des contrôles de validité, mesure et inclusion–exclusion : il faut préserver cet investissement. Ajouter progressivement miroirs, permutations des outils, reparamétrages, changements d’échelle et positions proches des contacts. [Harnais existant](../../../tests/fuzz/boolean.rs).

**Critère de réussite.** Chaque changement du moteur peut être comparé à une base reproductible : évolution des refus par famille, zéro nouveau résultat faux, aucun cas perdu parce que le programme ne compile plus. Le succès de compilation et la réussite des cas restent deux résultats distincts.

### P1 — Rendre les budgets et les garanties cohérents de bout en bout

**Action.** Auditer d’abord les deux points localisés : propagation complète des options vers le rayon NURBS et conversion de la marge UV en distance géométrique cohérente. Distinguer explicitement tolérance de décision, erreur d’approximation atteinte et couverture de la recherche. Renforcer les contrôles de pcurves et d’ajustement par validation adaptative et, là où nécessaire, par bornes conservatrices.

Exemple de l’écart entre budgets actuels : pour une échelle résolue de 100 unités et les valeurs par défaut, `linear = 10⁻⁷`, tandis que `section_fit ≈ 3,16 × 10⁻³`. Ce n’est pas automatiquement une erreur : un ajustement peut avoir un budget différent d’un test d’incidence. En revanche, le réseau et les étapes suivantes doivent savoir quel budget l’approximation garantit réellement. [Formules](../../../src/builders/boolean/tolerance.rs), lignes 68–78 ; [valeurs par défaut](../../../src/geometry/dim3/intersections/options.rs), lignes 56–73.

**Critère de réussite.** Une opération à budget fixé garde le même sens après changement de représentation ou d’échelle ; les écarts permis sont observables et vérifiables. L’ajout d’un outil éloigné ne dégrade pas silencieusement la résolution d’un petit détail local. Ce dernier risque est une hypothèse à tester, déduite du calcul sur l’étendue combinée.

**Inspiration.** OCCT pour l’erreur effectivement atteinte et les reprises d’incidence ; SMLib pour la séparation des rôles de tolérances. NGK peut conserver un budget d’acceptation immuable tout en transportant des incertitudes locales : copier une croissance implicite des tolérances n’est pas nécessaire.

### P1 — Résoudre les régions communes courbes et les contacts dégénérés

**Action.** Définir un résultat interne explicite pour le support commun et sa correspondance paramétrique, puis résoudre le recouvrement des domaines rognés dans cet espace. Commencer par les supports analytiques identifiables, tout en gardant le contrat utilisable par les NURBS généraux. En parallèle, distinguer tangence ponctuelle, tangence le long d’une courbe, coïncidence de région et voisinage trop mal conditionné pour conclure.

**Critère de réussite.** Une face cylindrique partiellement recouverte par une copie, une copie miroir et une représentation reparamétrée donne le même ensemble attendu, avec sections et orientations cohérentes. Les cas hors domaine sont refusés avec la zone et la cause précises.

**Inspiration.** Blocs communs et même domaine d’OCCT ; proto-topologie et secteurs de SMLib. Le réseau NGK est le point d’intégration naturel.

### P1 — Autoriser zéro, un ou plusieurs solides

**Action.** Concevoir un résultat booléen contenant une collection de solides et leur filiation. Le vide devient un résultat normal. Distinguer plusieurs solides réguliers d’une frontière non manifold ; accepter les premiers n’oblige pas à accepter tous les seconds. Une commodité exigeant un seul solide peut rester explicite au niveau approprié.

**Critère de réussite.** Union de deux boîtes séparées, intersection disjointe et soustraction séparant une pièce en deux sont exprimables. Les cavités restent rattachées à la bonne enveloppe extérieure. Le changement concerne les builders, la façade de modélisation et les bindings ; il mérite une décision d’API avant implémentation.

**Gain.** Couverture fonctionnelle immédiate sans prétendre résoudre un problème numérique. C’est le chantier le plus clairement indépendant de la qualité des intersections.

### P2 — Consolider les domaines périodiques et les relations locales

**Action.** Faire converger le parcours des domaines rognés utilisé par booléens, mesures et validation, en s’appuyant sur le mémo de refactor existant. Ajouter des cas de coutures, pôles, trous traversant une période et fragments étroits. Formaliser les secteurs et changements d’appartenance au contact ; n’utiliser une propagation entre fragments qu’avec des barrières complètes et validées.

**Critère de réussite.** Décaler une couture ou changer une paramétrisation ne modifie ni le volume ni l’appartenance du résultat. La classification locale et le contrôle global se corroborent sur les cas limites.

### P3 — Optimiser les coûts mesurés

**Action.** Mesurer séparément préparation/BVH, intersections, finalisation du réseau, clipping, découpe, classification, assemblage, healing et validation. Relever nombre de couples, subdivisions, itérations, fragments et mémoire. Examiner ensuite réutilisation des préparations entre passes, réduction des rayons, partition réutilisable et parallélisation déterministe des observations indépendantes.

**Critère de réussite.** Une optimisation améliore une famille identifiée sans augmenter les refus ou les résultats faux, sur le même niveau d’erreur autorisé. Les coûts de transaction et de copie sont inclus ou isolés explicitement.

OCCT fournit un exemple instructif : une correction de validation courbe/surface annonce 17–20 % de réduction sur deux charges propres au projet, en évitant des optimisations inutiles. Cela suggère de profiler la chaîne entière ; ce chiffre n’est pas transférable à NGK. [PR OCCT #1373](https://github.com/Open-Cascade-SAS/OCCT/pull/1373).

## 11. Protocole pour une vraie comparaison expérimentale

### Corpus commun

| Groupe | Cas à inclure |
| --- | --- |
| Solides simples | Boîtes, cylindres, sphères, cônes, tores ; disjoints, emboîtés, traversants. |
| Contacts | Face commune complète/partielle, tangence face/arête/sommet, quasi-contact autour de la tolérance. |
| Périodiques | Coupes traversant coutures et pôles, décalage de la couture, boucle intérieure fermée. |
| NURBS | Surfaces réellement libres, plusieurs branches, contacts singuliers, supports identiques reparamétrés. |
| Multi-outils | Outils recouvrants, répétitions, permutations, pont créé par un outil ultérieur, contacts triples. |
| Échelles | Même scène petite/grande, grandes coordonnées, petit détail dans de grands opérandes. |
| Modèles échangés | Corpus STEP séparé, avec diagnostic d’import et de healing avant le booléen. |

Construire d’abord les scènes natives évite de mesurer surtout la différence entre importeurs. Les tests STEP mesurent ensuite la chaîne d’usage réelle. Conserver les géométries, unités, versions, options et transformations exactes de chaque cas.

### Contrôles et métriques

Pour les cas analytiques, utiliser une attente calculable indépendamment. Pour les autres, croiser plusieurs contrôles ; prendre OCCT ou SMLib comme unique vérité masquerait leurs propres défauts.

- Validité combinatoire, fermeture, orientation et composantes attendues.
- Accord continu ou borné, selon le contrôle, entre sections 3D et pcurves.
- Appartenance de points distribués dans l’espace et près des frontières, avec traitement explicite des zones indécidables.
- Volumes, centres de gravité et identités ensemblistes ; le volume seul peut manquer un défaut local.
- Métamorphismes : échange des opérandes pour union/intersection, ordre des outils, rotations, miroirs, échelles, reparamétrages.
- Temps et mémoire par phase, médiane et queue de distribution, temps des refus compris.

Présenter séparément : **taux de résultats corrects**, **taux de refus**, **taux de résultats faux**, **crashes**, **timeouts**, puis performances des cas corrects communs. Un moteur qui refuse vite ne doit pas apparaître comme le plus rapide ; un moteur qui poursuit avec un avertissement ne doit pas être automatiquement compté comme correct.

Fixer un niveau d’erreur géométrique comparable plutôt que le même nombre dans des paramètres aux sens différents. Mesurer une exécution à un thread, puis une configuration parallèle explicite. Adapter les sémantiques multi-outils avant de lancer les cas.

## 12. Orientation recommandée

**La direction proposée est de conserver l’architecture NGK et de rendre ses décisions locales plus cohérentes, mieux bornées et mieux mesurées.** OCCT est la référence la plus utile pour le domaine fonctionnel, la consolidation et l’exploitation d’une partition commune. SMLib apporte une autre lecture particulièrement intéressante des contacts et de leur classification. Leurs défauts publics montrent pourquoi NGK doit viser simultanément une meilleure couverture et une détection fiable de ses propres limites.

Le premier ensemble de travail concret est donc : **fuzzing opérationnel → cohérence des budgets et garanties → recouvrements courbes/tangences**, avec **résultats multiples** comme extension d’API à fort rendement. Les optimisations se choisissent ensuite à partir de profils et d’un corpus qui distingue réellement résultat correct, refus et résultat faux.
