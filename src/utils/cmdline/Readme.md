
## Thematize_directory.bash

This tiny shell script is used in order to build the ad-hoc CityGML file hierarchy expected by the some of 3dUse's plugin (like the [Sunlight Plugin](../../doc/SunlightPlugin/UserGuide.md#prerequisites) or the Visibility Plugin.
In takes as input a directory holding sub-directories with [Lyon's CityGML](https://data.grandlyon.com/) (make a search with the CityGML key) borough data with the naming schema used when you download and unzip them: the general form of the name is `<Borough_name>_<category>_<vintage_year>.gml` (where `category` ranges within BATI, PONT, EAU...) e.g. `BRON_BATI_2009.gml`.

The script usage goes:
```
./Thematize_directory.bash <directory_to_thematize>
```
and the result will be a newly created sub-directory of the current directory
named as the source directory postponed with a "-Thematized".

Not that this script leaves the source directory unchanged: it copies the required files to the newly created thematized directory
