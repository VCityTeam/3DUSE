# Workspace ADE

Management of workspaces in CityGML.
In a team involving several researchers from different domains proposing various hypothesis and future plans using CityGML files, one important requirement is to be able to save, track and manage these propositions. 
Another major requirement is the ability to easily exchange these propositions with other researchers in an interoperable manner. 
WorkspaceADE is a step towards this direction.
It permits the researchers to propose their hypothesis and urban plans using CityGML and also be able to easily exchange them with each other.
This page explains the evolution of this Workspace ADE proposition.
Note that, we want the approach to be generic enough so that we can deploy other options like distributed version control systems, semantic web technologies to test this proposition.

## Evolution of Workspace ADE
### Proposition v0.1 
#### Description
It makes use of [CSG16] to propose workspaces.
[CSG16] suggested making use of 'tags' to create user specific workspaces for hypothesis.
Our proposition is to add a new class to represent the details of workspaces like name, owner and, creation date.
Workspaces are owned by individual users.
A workspace called 'Main' is used as a reference workspace.
Other workspaces may propose any changes from the Main workspaces.
In order to track a workspace, two pointers are used, the first version and the current version.
First version corresponds to the starting version of the workspace.
Current Version points to the latest version on a given workspace.

![Image Alt](WorkspaceADE.png)
![Image Alt](WorkspacePropositionv0.1.png)

#### Limitations
* Unable to distinguish between hypothesis (past) and future urban plans
* Unable to distinguish between different types of transitions proposed in [CSG16]

[CSG16] C HATURVEDI K., S MYTH C. S., G ESQUIÈRE G., K UTZNER T., K OLBE T. H.: Managing versions and history within semantically enriched 3d city models. Advances in 3D Geoinformation, Lecture Notes in Cartography and Geoinformation, Springer, 2016


### Proposition v0.2 
#### Description
Add  the notion of past and future considering the current time
* Hypothesis concerns the past
* Plans concerns the future

Add different types of version transitions
* Regular Transition (Transitions occurring in the main workspace)
* Proposed Historical Succession (A transition from a version of the main workspace to a version used to describe hypothesis)
* Proposed Historical Regular Transition (Transition between versions used to describe hypothesis)
* Validated (A transition from the version used to describe hypothesis to a version in the Main workspace)
* Planned (A transition from a version in the main workspace to a version used to describe future plans)
* Planned Regular Transition (Transitions between versions used to describe future plans)
* Realized (A transition from the version used to describe future plans to a version in the Main workspace)

![Image Alt](WorkspacePropositionv0.2.png)

#### Limitations
* Too many types of version transitions (complex).
* Missing notion to represent when any proposed future plans impact something in the main workspace 

### Proposition v0.3 
![Image Alt](WorkspacePropositionv0.3.png)
### Proposition v0.4 
![Image Alt](WorkspaceVCSComparison.png)
![Image Alt](Git.png)
![Image Alt](WorkspacePropositionv0.4.png)
![Image Alt](WorkspacePropositionv0.4Workspaces.png)
### Proposition v0.5 
![Image Alt](WorkspacePropositionv0.5Workspaces.png)
![Image Alt](WorkspacePropositionv0.5Hypothesis.png)
![Image Alt](WorkspacePropositionv0.5Planned.png)
![Image Alt](WorkspacePropositionv0.5Influence.png)
![Image Alt](WorkspacePropositionv0.5Influence2.png)
![Image Alt](WorkspacePropositionv0.5Quality.png)
### Proposition v0.6 
![Image Alt](WorkspacePropositionv0.6Hypothesis.png)
![Image Alt](WorkspacePropositionv0.6Influence.png)
![Image Alt](WorkspacePropositionv0.6Influence2.png)
![Image Alt](WorkspacePropositionv0.6Quality.png)
![Image Alt](WorkspacePropositionv0.6TimeDiagram.png)
![Image Alt](WorkspacePropositionv0.6IntervalDiagram.png)
### Proposition v0.7 
![Image Alt](WorkspacePropositionv0.7Past.png)
![Image Alt](WorkspacePropositionv0.7TimeDiagram.png)
![Image Alt](WorkspacePropositionv0.7IntervalDiagram.png)


