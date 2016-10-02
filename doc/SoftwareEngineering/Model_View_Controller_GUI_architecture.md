FIXME: translate this to English and move it to Doxygen

Le projet est basé autour du pattern MVC. On retrouve alors 3 grands composants : le Modèle, la Vue et le Controlleur.

## Modèle
Le modèle contient tout ce qui se trouve dans le dossier core et regroupé dans le namespace vcity.

#### Application
Permet de stocker et d’accéder aux principaux éléments du programme :
- la scène
- les paramètres
- le controlleur
- les algorithmes
- la liste des noeuds sélectionnées.

#### Settings
Stoque les paramètres de l’application et le profil de données actif.
Contient le paramètre permettant de charger ou non les textures.

#### DataProfile
Contient les paramètres permettant de positionner des données dans l’espace : 
- bounding box indiquant la zone couverte par les données (utilisé pour dessiner la grille)
- taille des tuiles en x et y
- offset pour décaller les données dans la vue 3D et limiter des problèmes de stabilité numérique liés à de grand nombres flottants (grande coordonnées en lambert93)
- offset pour les id de tuile (pour Paris, la première tuile est : 1286, 13714 )

Quelques méthodes static sont fournis pour créer des profils de données prédéfinis, pour Paris et Lyon.

#### Scene
Stocke principalement les layers, mais fournit aussi des mécanismes de recherche de noeuds grâce aux URI.

#### abstractLayer
layer de haut niveau, abstrait. Possède un nom. On peut récupérer une URI pointant sur ce layer avec la méthode getURI.

#### LayerCityGML
Layer pouvant stocker des tuiles de données CityGML. Fourni des méthdes pour accéder aux tuiles et aux noeuds CityGML avec des fonctions de recherche par URI.

#### Tile
Une tuile est un conteneur pour un CityModel.

#### URI 
c’est un mécanisme d’adressage de noeud. Une URI vers un noeud est la concatéation du nom des noeuds à traverser pour accéder à ce noeuds. Ressemble à un chemin vers un fichier.
Ce système d’adressage est utilisé à travers tout le programme et permet de synchroniser la vue et le modèle. Cela se fait principalement dans le ControlleurGui.
Une URI possède un curseur qui est au départ au début de l’URI (racine). On peut faire avancer ce curseur avec popFront. On peut le remettre au début avec resetCursor.
Ce mécanisme est en fait assez lent, car toute les recherche de noeud sont des comparaisons de string. Tout cela pourrait être amélioré en stoquant un hachage à la place des strings, la contrainte étant de pouvoir conserver le lien entre les 3 arbres : treeview, scene et osgscene pour permettre la synchronisation, picking.

#### LayerMNT, LayerSHP, LayerAssimp
ces layers permettent respectivement des stocker des données MNT (mnt au format asc de l’IGN), des shape, des données pouvant être lues par Assimp (ex : obj).

#### Algo 
classe regroupant les codes de calcul de Frédéric. Permet de faire du calcul de LOD, de détection de changement. Ce module pourrait être découpé en plusieurs sous-modules ou réorganisé en une boite à outils.

## Vue
La vue contient ce qui se trouve dans le dossier gui et contient le code Qt pour l’interface, le code OSG pour l’affichage 3D et le code permettant de convertir les données source (CityGML, shp, …) en données affichable par OSG.
Les fichiers de code d’interface Qt commence par dialog et sont couplé à un fichier d’interface .ui éditable graphiquement avec QtCreator.

#### ApplicationGui
Stoque les éléments liés à l’interface graphique :
- la scene OSG
- le controlleur graphique (ControlleurGui)
- le treeview (pour la gestion de l’arbre à gauche de la fenêtre)

#### OsgQtWidget
Interface entre OSG et Qt pour y insérer un widget 3D. Contient la gestion des évenements / caméra pour la vue 3D.

#### PickHandler
Gestion du picking dans OSG. Les noeuds pickés sont stockés dans la liste d’URI contenue dans la classe Application.
OSG permet de faire du picking avec osgUtil::LineSegmentIntersector. OSG retourne toutes les intersections trouvées sur le rayon. GetFirstIntersection permet de prendre la première, ce qui nous intéresse. Les éléments sélectionnés sont des nœuds osg. Dans notre représentation des données CityGML avec OSG, chaque élément CityGML correspond à un nœud OSG. Cela permet donc de sélectionner un mur, mais pas directement un building complet. Pour cela, il faut remonter dans l'arbre osg pour trouver le nœud correspondant au building (mode picking face / bâtiment).
Possibilité de picking par rectangle de sélection avec osgUtil::PolytopeIntersector.

#### OsgScene
La scène OSG. Elle stoque les noeuds affichable en 3D. Copie de la structure de l'arbre CityGML. Parcours de l'arbre CityGML. A chaque nœud rencontré, création d'un nœud osg et création de la géométrie associée.

#### OsgCityGML
Contient des outils pour lire des données CityGML et les convertir en un noeud OSG.

#### OsgGDAL, OsgAssimp, OsgMnt
Contient des outils pour lire des données shape, obj, mnt et les convertir en noeud OSG.

#### TreeView
Permet la gestion de l’arbre sur la gauche de la fenêtre. Vue hiérarchique des éléments chargés. Représentation visuelle de ce qui est chargé en mémoire.
Outils : clic droit sur un nœud : menu contextuel en fonction du type de nœud.
Ex : Building → éditer → modifier la position
TAG → éditer → modifier la date

#### MainWindow
Ce module relie tous les éléments de l’interface : il initialise la fenêtre, la scène OSG, le treeview. Contient les fonctionnalités de chargement de fichiers, de fichiers récents et d’accès à toutes les fonctionnalités du programme.

## Controlleur

#### ControllerGui et Controlleur
Le controlleur graphique utilise les actions du controlleur classique. ex : reset des noeuds séléctionnés
````
    void ControllerGui::resetSelection()
    {
      // reset in treeview
      appGui().getTreeView()->resetSelection();
      // reset in osg scene
      appGui().getPickHandler()->resetPicking();
      // must be done last
      Controller::resetSelection();
    }
```
3 actions :
- reset selection dans le treeview
- reset selection dans le picking OSG, pour que les noeuds ne soient plus surbrillé
- appel du controlleur classique qui vide la liste d’URI dans la classe Application

Autre exemple : Action sur les nœuds et répercussions dans les autres arbres :
deleteNode : il faut synchroniser les 3 arbres.
On retrouve un nœud dans un arbre grâce à une URI (unique dans la scène)
On utilise cette URI pour faire la même action sur les 3 arbres. (permet d'éviter de lier les éléments des différents arbres par pointeur, difficile à maintenir)

