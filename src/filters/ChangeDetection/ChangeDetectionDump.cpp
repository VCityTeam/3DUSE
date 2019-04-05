// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

// This file holds the functions for interpreting the output from ChangeDetection
// filter to create a graphML-JSON file expressing the changes between the
// buildings of the two vintages provided in input. The entrance point is the
// function DumpIDCorrespondancesJson().

#include <stdlib.h>                      // exit, EXIT_FAILURE
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace pt = boost::property_tree;

#include "ChangeDetectionDump.hpp"
#include "ChangeDetection.hpp"

// Denotes a single correspondence between buildings (i.e. a change).
// It is composed of a change status and a building id which
// corresponds to the target of the correspondence. It will
// be mapped to the id of the building source of the correspondence
// in buildingCorres and reverseBuildingCorres maps
struct singleCorrespondence
{
    public:
        int change_status;
        std::string building_id;
};

// Holds multiple "single correspondence" and will be mapped to a building
// to express all of its correspondences
typedef std::vector<singleCorrespondence> Correspondences;


// The global (accross all time stamps) identifier is a string with
//  - a time stamp followed by
//  - a double colon ("::")
//  - the GmlId of some building
std::string to_global_id(int time_stamp, std::string gmlId )
{
  return   boost::lexical_cast<std::string>(time_stamp)
         + "::"
         + gmlId;
}

// Adds a singleCorrespondence to the corresponceMap (maps each building to a Correspondences
// object)
void AddSingleCorrespondenceToMap(std::map<std::string, Correspondences> &correspondenceMap,
                            std::string building_id,
                            singleCorrespondence &correspondence)
{
    // If there is no entry in the map for the building_id
    if (correspondenceMap.count(building_id) == 0)
    {
      // Create a correspondences object and add the correspondence to it
      Correspondences buildingCorrespondences;
      buildingCorrespondences.push_back(correspondence);
      // Map the building_id to the new Correspondences object holding the correspondence
      // passed as an argument
      correspondenceMap.emplace(building_id, buildingCorrespondences);
    }
    // else, add the correspondence to the correspondences already mapped to the
    // building_id passed as parameter
    else {
      correspondenceMap[building_id].push_back(correspondence);
    }
}

// Fills the buildingsCorrespondences and reverseBuildingsCorrespondences maps from the
// change object.
// change is the output structure from ChangeDetection filter mapping the changes between
// building footprints
// buildingsCorrespondences and reverseBuildingsCorrespondences map the changes between
// "semantical buildings" (i.e. identified through their global_id) respectively of
// time_stamp1 and of time_stamp2
void fillCorrespondencesMaps(ChangeDetectionRes &change,
                             int time_stamp1,
                             int time_stamp2,
                             std::map<std::string, Correspondences> &buildingsCorrespondences,
                             std::map<std::string, Correspondences> &reverseBuildingsCorrespondences)
{

    int nbrInitialBuilding = change.Compare->first.size();
    for (int building_index = 0;
             building_index < nbrInitialBuilding;
             building_index++)
    {
        // compute global id of source building
        std::string source_global_id = to_global_id(time_stamp1,
                                             (*change.BuildingID1)[building_index]);

        // get object correspondences
        auto correspondences = change.Compare->first[building_index];

        std::size_t correspondences_length = correspondences.size();

        if ( correspondences_length == 0 )
            // no correspondence to store
            continue;

        // The building has one or possibly many associated future buildings
        // but the encoding is a succession of (change_status, building_new_index)
        // pairs and must thus have an even length.
        if( ! correspondences_length %2 )
        {
          std::cout << "ERRONEOUS encoding of Compare (should always be even) "
                    << "   of building with id: "
                    << building_index
                    << std::endl;
          continue;
        }

        // We have one or more building correspondents.
        // Iterate over correspondences to fill the buildingsCorrespondences
        // and reverseBuildingsCorrespondes maps
        for (std::size_t i = 0; i < correspondences_length; i += 2)
        {
            int change_status =  change.Compare->first[building_index][i];
            int new_index     =  change.Compare->first[building_index][i+1];
            std::string target_global_id = to_global_id(time_stamp2,
                                               (*change.BuildingID2)[new_index]);

            // Create the current correspondence
            singleCorrespondence currentCorres;
            currentCorres.change_status = change_status;
            currentCorres.building_id = target_global_id;

            // Fill the buildingsCorrespondences map with it
            AddSingleCorrespondenceToMap(buildingsCorrespondences, source_global_id, currentCorres);

            // Create the reverse correspondence
            singleCorrespondence reverseCorres;
            reverseCorres.change_status = change_status;
            reverseCorres.building_id = source_global_id;

            // Fill the ReverseBuildingsCorrespondences map with it
            AddSingleCorrespondenceToMap(reverseBuildingsCorrespondences, target_global_id, reverseCorres);
        }
    }
}

// this function maps the buildings id to their area. This will be used to determine
// fusions and subdivision
// note that it could be refactored in one function and called for time_stamp1 and for timestamp2
std::map<std::string, double> ComputeBuildingAreas(ChangeDetectionRes &change,
                                                int time_stamp1,
                                                int time_stamp2) {

    // first time_stamp
    std::map<std::string, double> buildingAreas;

    // start by the initial buildings (i.e. from timestamp 1)
    int nbrInitialBuilding = change.EnveloppeCityU1->getNumGeometries();
    for (int building_index = 0;
             building_index < nbrInitialBuilding;
             building_index++)
    {
        // compute area of current building footprints
        OGRPolygon * currentBuilding = (OGRPolygon *)change.EnveloppeCityU1->getGeometryRef(building_index)->clone();
        double area = currentBuilding->get_Area();

        // compute global id corresponding to the current building footprints
        std::string building_global_id = to_global_id(time_stamp1,
                                             (*change.BuildingID1)[building_index]);

        // If there is no entry in the map for the building_global_id
        if (buildingAreas.count(building_global_id) == 0)
        {
            // Map the buildingAreas to the Area of the current building fooprints
            buildingAreas.emplace(building_global_id, area);
        }
        // else (building_global_id corresponds to a building having several parts
        // which are not in a connected space).
        else {
            // Add the area of the current building footprints to already
            // computed area
            buildingAreas[building_global_id] += area;
        }
    }

    // second time_stamp
    // continue with the final buildings (i.e. from timestamp 2)
    int   nbrFinalBuilding = change.EnveloppeCityU2->getNumGeometries();
    for (int building_index = 0;
             building_index < nbrFinalBuilding;
             building_index++)
    {
        // compute area of current building footprints
        OGRPolygon * currentBuilding = (OGRPolygon *)change.EnveloppeCityU2->getGeometryRef(building_index)->clone();
        double area = currentBuilding->get_Area();

        // compute global id corresponding to the current building footprints
        std::string building_global_id = to_global_id(time_stamp2,
                                             (*change.BuildingID2)[building_index]);

        // If there is no entry in the map for the building_global_id
        if (buildingAreas.count(building_global_id) == 0)
        {
            // Map the buildingAreas to the Area of the current building fooprints
            buildingAreas.emplace(building_global_id, area);
        }
        // else (building_global_id corresponds to a building having several parts
        // which are not in a connected space).
        else {
            // Add the area of the current building footprints to already
            // computed area
            buildingAreas[building_global_id] += area;
        }
    }

    return buildingAreas;
}

/*
 * This function compares the change statuses of each singleCorespondence
 * from the argument correspondences and returns:
 *   * 0 if correspondences is empty
 *   * -1 if all change statuses are -1
 *   * -2 if all change statuses are -2
 *   * -3 if change statuses are a mix of -1 and -2
 */
int compareChangeStatuses(const Correspondences &correspondences) {

    if(correspondences.size() == 0)
        return 0;

    int unified_change_status = correspondences[0].change_status;

    for(int i = 1; i < correspondences.size() ; ++i)
    {
        // if change status is not the same as the first/the previous change
        // statuses, then it is a mix and we return -3
        if(correspondences[i].change_status != unified_change_status)
            return -3;
    }

    return unified_change_status;
}

/*
 * This function initialize an edge of the graph with an id, a source and a
 * target.
 */
pt::ptree initEdge(int edge_id, int source_node_id, int target_node_id) {

    pt::ptree edge;

    edge.put("id", boost::lexical_cast<std::string>(edge_id)); // TODO: assert cast is needed
    edge.put("source", source_node_id);
    edge.put("target", target_node_id);
    edge.put("type", "replace"); // TODO: remove type

    return edge;
}

/*
 * This function takes the result of the ChangeDetection algorithm
 * and focuses on the correspondences between buildings seen through
 * their GmlId (the ID found in CityGML). Because this identifier is
 * not unique across different time stamps, a general identifier is
 * build out of the input time stamps (refer to to_global_id() function).
 * At this level of abstraction (where geometry is set aside) the
 * correspondance between buildings considered at two time stamps can
 * be seen as a graph where the nodes are the building identifiers and
 * the edges the correspondences between old and new building.
 * Such a graph can be expressed in THE de facto standard graph format
 * that is GraphML. Nevertheless we here chose to use the Json adaptation
 * of  GraphML instead of its standard XML format.
 * Refer to e.g.
 *   https://github.com/gregors/graphml2json
 *   https://github.com/uskudnik/GraphGL/blob/master/examples/graphml-to-json.py
 * If you want true Graphml, use boost's ptree::write_xml() instead of
 * ptree::pt::write_json()
 */
void DumpIDCorrespondancesJson(ChangeDetectionRes change,
                               int time_stamp1,
                               int time_stamp2,
                               std::string filename,
                               pt::ptree& comment)
{
  // Concerning the file format:
  // - GraphML is the standard for graphs
  // - Parsing graphml files in Python is done with the that is GPL
  // - Parsing JSON with Python comes for free
  // - we choose a Json based format that is close to GraphML abstract structure
  //   as done with e.g.
  //     https://github.com/uskudnik/GraphGL/blob/master/examples/graphml-to-json.py
  //   or
  //     https://github.com/gregors/graphml2json
  //   Reference: some graphml (XML) example files look here
  //     https://gephi.org/users/supported-graph-formats/graphml-format/
  //
  // Note concerning boost::ptree's write_json is well known for not conforming
  // to JSON standard: refer to e.g.
  //  - https://svn.boost.org/trac10/ticket/9721
  //  - https://stackoverflow.com/questions/2855741/why-boost-property-tree-write-json-saves-everything-as-string-is-it-possible-to
  // The undesired side effect is that Node and Edge ids will be serialized as
  // json strings (e.g. "45") as oppose to json integers (e.g. 45)
  //
  // Concerning the output attributes, see:
  // https://github.com/MEPP-team/VCity/wiki/DesignNote037#ouput-of-extractbuildingsconstructiondemolitiondates
  pt::ptree graph;
  graph.add_child("_comments", comment);

  //////////////////  Start by dumping the nodes
  int node_index = 0;
  typedef std::map< std::string, int> ReverseNodeIndexType;
  ReverseNodeIndexType ReverseNodeIndex;

  pt::ptree nodes;

  // And first with register the nodes corresponding to the first time_stamp1
  int nbrInitialBuilding = change.EnveloppeCityU1->getNumGeometries();
  for (int building_index = 0;
           building_index < nbrInitialBuilding;
           building_index++)
  {
    std::string global_id = to_global_id(time_stamp1,
                                          (*change.BuildingID1)[building_index]);
    // Check if this global id is already in the map (i.e. in the nodes of the graph) to avoid
    // duplications. This can happen because, at the begining of ChangeDetection,
    // "semantic buildings" as defined in CityGML are split in parts which are in a connected space.
    // Hence, if a "semantic building" (having one ID) is split into multiple parts,
    // each part will have the the same ID (of the original "semantic building").
    if (ReverseNodeIndex.count(global_id) == 0) {
        ReverseNodeIndex.emplace(global_id, node_index);
        pt::ptree node;
        node.put("id", boost::lexical_cast<std::string>(node_index++));
        node.put("globalid", global_id);
        nodes.push_back(std::make_pair("", node));
    }
  }

  // Then register the nodes corresponding to the second time_stamp1
  int   nbrFinalBuilding = change.EnveloppeCityU2->getNumGeometries();
  for (int building_index = 0;
           building_index < nbrFinalBuilding;
           building_index++)
  {
    std::string global_id = to_global_id(time_stamp2,
                                         (*change.BuildingID2)[building_index]);
    if (ReverseNodeIndex.count(global_id) == 0) {
        ReverseNodeIndex.emplace(global_id, node_index);
        pt::ptree node;
        node.put("id", boost::lexical_cast<std::string>(node_index++));
        node.put("globalid", global_id);
        nodes.push_back(std::make_pair("", node));
    }
  }

  // We are done with the nodes.
  graph.add_child("nodes", nodes);

  //////////////////  Proceed with the edges

  // Create two maps to ease the creation of the edges:
  //   * buildingCorrespondences which maps the id of each building from time_stamp1 to its
  //     correspondences with buildings from time_stamp2 expressed as a Correspondences object
  //   * reverseBuildingsCorrespondences which maps the id of each building from time_stamp 2 to its
  //     correspondences with buildings from time_stamp1 expressed as a Correspondences object
  // However, note that buildings having no correspondences with are not sotred as they won't have any edge generated
  std::map<std::string, Correspondences> buildingsCorrespondences;
  std::map<std::string, Correspondences> reverseBuildingsCorrespondences;

  fillCorrespondencesMaps(change, time_stamp1, time_stamp2,
                          buildingsCorrespondences, reverseBuildingsCorrespondences);

  // buildings area are used for subdivision and fusions
  std::map<std::string, double> buildingAreas = ComputeBuildingAreas(change, time_stamp1, time_stamp2);
  double areas_comparison_epsilon = 2;

  int edge_index = 0;
  ReverseNodeIndexType::iterator iter;
  pt::ptree edges;

  // iterate through the buildings having correspondences to create the
  // associated edges
  for (const auto& OneBuildingCorrespondences : buildingsCorrespondences)
  {
      std::string buildingId = OneBuildingCorrespondences.first;
      Correspondences buildingCorres = OneBuildingCorrespondences.second;
      std::size_t correspondence_length = buildingCorres.size();

      if (correspondence_length == 0)
      {
          std::cout << "Correspondence map should only store buildings with correspondences"
                    << std::endl;
          exit (EXIT_FAILURE);
      }

      // Get source_node_id which is the id of the node of the graph
      // corresponding to buildingId
      iter = ReverseNodeIndex.find(buildingId);
      if (iter == ReverseNodeIndex.end())
      {
        std::cout << "Unfound source id within ReverseNodeIndex: "
                  << buildingId
                  << std::endl;
        exit (EXIT_FAILURE);
      }
      int source_node_id = iter->second;

      if (correspondence_length == 1)
      {
          int change_status = buildingCorres[0].change_status;
          std::string targetBuildingId = buildingCorres[0].building_id;

          // Get target_node_id which is the id of the node of the graph
          // corresponding to targerBuildingId
          iter = ReverseNodeIndex.find(targetBuildingId);
          if (iter == ReverseNodeIndex.end())
          {
              std::cout << "Unfound target id within ReverseNodeIndex: "
                        << targetBuildingId
                        << std::endl;
              exit (EXIT_FAILURE);
          }
          int target_node_id = iter->second;

          Correspondences reverseBuildingCorres = reverseBuildingsCorrespondences[targetBuildingId];
          std::size_t reverse_correspondence_size_length = reverseBuildingCorres.size();

          if(reverse_correspondence_size_length == 1)
          {
              if (change_status == -1)
              {
                  pt::ptree edge = initEdge(edge_index++, source_node_id, target_node_id);

                  if(buildingId == targetBuildingId)
                      // Unchanged building: same geometry, same (gml) ID.
                      edge.put("tags", "unchanged");
                  else
                      // The building has been re-ided: same geometry, different (gml) ID
                      edge.put("tags", "re-ided");

                  edges.push_back(std::make_pair("", edge));
                  continue;
              }
              else if (change_status == -2)
              {
                  pt::ptree edge = initEdge(edge_index++, source_node_id, target_node_id);

                  // The building geometry and ID have changed
                  // e.g. it was heightened; its footprint geometry changed, etc.
                  edge.put("tags", "modified");

                  edges.push_back(std::make_pair("", edge));
                  continue;
              }
              else
              {
                  std::cout << "ERRONEOUS change status: must be -1 or -2."
                            << std::endl;
                  exit (EXIT_FAILURE);
              }
          }
          else // reverse_correspondence_size_length > 1
          {
              // In this case we need to check the change status of each singleCorrespondence in
              // reverseBuildingCorres. It can be:
              //   * multiple -1 which indicates a split building that didn't work for both timestamps
              //     and which then denotes a logical fusion
              //   * multiple -2 only which indicates a fusion or a split building problem + modifications
              //   * multiple -2 and -1 which indicates a split building problems + modifications
              int change_statuses_comparison = compareChangeStatuses(reverseBuildingCorres);

              std::string edge_tag;
              if (change_statuses_comparison == -1)
                  edge_tag = "fused";
              else if (change_statuses_comparison == -2)
              {
                  // Compute building area of source buildings by looping on reverseBuildingCorres
                  double source_buildings_area = 0;
                  for(int i = 0; i < reverse_correspondence_size_length; ++i)
                  {
                      std::string currentSourceBuildingId = reverseBuildingCorres[i].building_id;
                      source_buildings_area += buildingAreas[currentSourceBuildingId];
                  }

                  // Compute building area of target building
                  double target_building_area = buildingAreas[targetBuildingId];

                  // if building areas are equal then fusion, otherwise: modified
                  if (fabs(target_building_area - source_buildings_area) < areas_comparison_epsilon)
                      edge_tag = "fused";
                  else
                      edge_tag = "modified";

              }
              else if (change_statuses_comparison == -3)
                  edge_tag = "modified";
              else
              {
                  std::cout << "ERRONEOUS change statuses comparison: must be -1, -2 or -3."
                            << std::endl;
                  exit (EXIT_FAILURE);
              }

              pt::ptree edge = initEdge(edge_index++, source_node_id, target_node_id);
              edge.put("tags", edge_tag);

              edges.push_back(std::make_pair("", edge));
              continue;
          }
      }
      else // current building has more than one correspondent
      {
          // In this case we need to check the change status of each singleCorrespondence in
          // buildingCorres. It can be:
          //   * multiple -1 which indicates a split building that didn't work for both timestamps
          //     and which then denotes a logical subdivision
          //   * multiple -2 only which indicates a subdivision or a split building problem + modifications
          //   * multiple -2 and -1 which indicates a split building problems + modifications
          int change_statuses_comparison = compareChangeStatuses(buildingCorres);

          std::string edge_tag;
          if (change_statuses_comparison == -1)
              edge_tag = "subdivided";
          else if (change_statuses_comparison == -2)
          {
              // Get building area of source building
              double source_buildings_area = buildingAreas[buildingId];

              // Get target buildings area by looping through buildingCorres
              double target_building_area = 0;
              for(int i = 0; i < correspondence_length; ++i)
              {
                  std::string currentTargetBuildingId = buildingCorres[i].building_id;
                  target_building_area += buildingAreas[currentTargetBuildingId];
              }

              // if building areas are equal then fusion, otherwise: modified
              if (fabs(target_building_area - source_buildings_area) < areas_comparison_epsilon)
                  edge_tag = "subdivided";
              else
                  edge_tag = "modified";

          }
          else if (change_statuses_comparison == -3)
              edge_tag = "modified";
          else
          {
              std::cout << "ERRONEOUS change statuses comparison: must be -1, -2 or -3."
                        << std::endl;
              exit (EXIT_FAILURE);
          }

          // create one edge per correspondent
          for (int i = 0; i < correspondence_length; ++i)
          {
              std::string targetBuildingId = buildingCorres[i].building_id;

              // Get ith target_node_id which is the id of the node of the graph
              // corresponding to targerBuildingId
              iter = ReverseNodeIndex.find(targetBuildingId);
              if (iter == ReverseNodeIndex.end())
              {
                  std::cout << "Unfound target id within ReverseNodeIndex: "
                            << targetBuildingId
                            << std::endl;
                  exit (EXIT_FAILURE);
              }
              int target_node_id = iter->second;

              pt::ptree edge = initEdge(edge_index++, source_node_id, target_node_id);
              edge.put("tags", edge_tag);

              edges.push_back(std::make_pair("", edge));
          }
      }
  }

  // We are done with the nodes.
  graph.add_child("edges", edges);

  pt::write_json(filename, graph);
}

/*
 * The role of this function is the same as DumpIDCorrespondancesJson
 * expcept that the output format is human oriented and sent to std::out
 * by default. DumpIDCorrespondancesDebug() is thus designed on debugging
 * purposes.
 */
void DumpIDCorrespondancesDebug(ChangeDetectionRes change,
                           int time_stamp1,
                           int time_stamp2)
{
  int nbrInitialBuilding = change.EnveloppeCityU1->getNumGeometries();
  int   nbrFinalBuilding = change.EnveloppeCityU2->getNumGeometries();
  for (int building_index = 0;
           building_index < nbrInitialBuilding;
           building_index++)
  {
    std::cout << (*change.BuildingID1)[building_index];
    // The correspondence (the first vector returned by CompareBati())
    // encodes what has become of an "old" building (the following is
    // a derived copy of an illustration encountered in CompareBati()):
    // correspondence[0]=[ -1, 0]: building 0 keeps same index and unchanged
    // correspondence[1]=[ -1, 2]: building 1 relabeled to become building 2
    // correspondence[2]=[ -1, 1]: building 2 relabeled to become building 1
    // correspondence[3]=[ -2, 4]: building 3 changed height and relabeled
    //                             to 4
    // correspondence[4]=[]:       building 4 disappeared
    // correspondence[5]=[ -2, 6, -2, 7]: building 5 was split into buildings
    //                                    6 and 7
    auto correspondence = change.Compare->first[building_index];
    std::size_t corresondence_length = correspondence.size();
    if ( corresondence_length == 0 )
    {
        std::cout << ": destroyed." << std::endl;
        continue;
    }
    // The building has one or possibly many associated future buildings
    // but the encoding is a succession of (change_status, building_new_index)
    // pairs and must thus have an even length.
    if( ! corresondence_length %2 )
    {
      std::cout << "ERRONEOUS encoding of Compare: should always be even!"
                << std::endl;
      continue;
    }

    if ( corresondence_length == 2 )
    {
       int change_status =  change.Compare->first[building_index][0];
       int new_index     =  change.Compare->first[building_index][1];
       auto new_id       =  (*change.BuildingID2)[new_index];

       if (change_status == -1)
       {
          if(building_index == new_index)
          {
             std::cout << ": unchanged (same geometry, same ID)."
                       << std::endl;
             continue;
          }
          else
          {
             std::cout << ": re-ided (same geometry, different ID) to ";
             std::cout << new_id
                       << std::endl;
             continue;
          }

       }
       else if (change_status == -2)
       {
          if(building_index == new_index)
          {
             // Debugging test
             std::cout << "DAMNED YOU CAN CHANGE AND KEEP YOUR ID !?"
                       << std::endl
                       << "INQUIRE ON THIS !"
                       << std::endl;
             continue;
          }

          std::cout << ": heightened (same footprint) to ";
          std::cout << new_id
                    << std::endl;
          continue;
       }
       else
       {
          std::cout << "ERRONEOUS change status: must be -1 or -2."
                    << std::endl;
          continue;
       }
    }

    // We have more than one correpondent. The original building was
    // thus either
    //   * split in sub-parts (while preserving its outside geometry)
    //   * revamped to new buildings (with a new total footprint included
    //     in the footprint of the original building): ASSERT THIS!

    std::cout << ": subdivided into " << std::endl;
    for (std::size_t j = 0; j < corresondence_length; j += 2)
    {
       int change_status =  change.Compare->first[building_index][j];
       int new_index     =  change.Compare->first[building_index][j+1];
       auto new_id       =  (*change.BuildingID2)[new_index];

       // Is it possible to have a change_status of -1 (geometry unchanged)
       // and still have many corresponding objects ?
       // (they should all be the same).
       if (change_status != -2)
       {
          // Debugging test
          std::cout << "ERROR: yes there is an occurence of a change "
                    << " status that is not -2 and its value is: "
                    << change_status
                    << std::endl;
          continue;
       }
       std::cout << "          " << new_id;
       if( j+2 < corresondence_length)
       {
          // Not the last item
          std::cout << ",";
       }
       else
       {
          // This is the last item
          std::cout << ".";
       }
       std::cout << std::endl;
    }
  }

  // Now deal with the new buildings that are to be found in the
  // the second list of geometries
  for (int building_index = 0;
           building_index < nbrFinalBuilding;
           building_index++)
  {
    // Among the buildings of the second geometries only the ones with
    // no correspondent in the first geometries are considered as new
    // ones.
    auto correspondence = change.Compare->second[building_index];
    if ( correspondence.size() == 0 )
    {
       auto new_id = (*change.BuildingID2)[building_index];
       std::cout << new_id
                 << " : Created."
                 << std::endl;
       continue;
    }
  }
}
