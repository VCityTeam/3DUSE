// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include <stdlib.h>                      // exit, EXIT_FAILURE
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
namespace pt = boost::property_tree;

#include "ChangeDetectionDump.hpp"
#include "ChangeDetection.hpp"


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
  int edge_index = 0;
  ReverseNodeIndexType::iterator iter;
  pt::ptree edges;

  for (int building_index = 0;
           building_index < nbrInitialBuilding;
           building_index++)
  {
    std::string source_id = to_global_id(time_stamp1,
                                         (*change.BuildingID1)[building_index]);
    iter = ReverseNodeIndex.find(source_id);
    if (iter == ReverseNodeIndex.end())
    {
      std::cout << "Unfound source id within ReverseNodeIndex: "
                << source_id
                << std::endl;
      exit (EXIT_FAILURE);
    }
    int source_node_id = iter->second;
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
    std::size_t correspondence_length = correspondence.size();
    if ( correspondence_length == 0 )
    {
        // The building was destroyed: no edge to declare (i.e. the node will
        // have no adjacent edge between time_stamp1 and time_stamp2)
        continue;
    }
    // The building has one or possibly many associated future buildings
    // but the encoding is a succession of (change_status, building_new_index)
    // pairs and must thus have an even length.
    if( ! correspondence_length %2 )
    {
      std::cout << "ERRONEOUS encoding of Compare (should always be even) "
                << "   of building with id: "
                << building_index
                << std::endl;
      continue;
    }

    int change_status = change.Compare->first[building_index][0];

    if (change_status == -1)
    {
        // The geometry of the building is unchanged
        if (correspondence_length == 2) {
            int new_index         =  change.Compare->first[building_index][1];
            std::string target_id = to_global_id(time_stamp2,
                                               (*change.BuildingID2)[new_index]);
            iter = ReverseNodeIndex.find(target_id);
            if (iter == ReverseNodeIndex.end())
            {
                std::cout << "Unfound target id within ReverseNodeIndex: "
                          << target_id
                          << std::endl;
                exit (EXIT_FAILURE);
            }
            int target_node_id = iter->second;

            pt::ptree edge;
            edge.put("id", boost::lexical_cast<std::string>(edge_index++));
            edge.put("source", source_node_id);
            edge.put("target", target_node_id);
            std::string building_GMLid = (*change.BuildingID1)[building_index];
            std::string      new_GMLid = (*change.BuildingID2)[new_index];

            if(building_GMLid == new_GMLid)
            {
              // The building is unchanged: same geometry, same (gml) ID.
              // Define an edge:
              edge.put("comment", "Unchanged: same geometry, same GML ID");
            }
            else
            {
              // The building was re-ided: same geometry, different (gml) ID
              // Define an edge:
              edge.put("comment", "Re-ided: same geometry, different GML ID");
            }
            edges.push_back(std::make_pair("", edge));
            continue;
        }
        else
        {
           // ERROR change status == -1 implies that there is only one correspondent
           std::cout << "ERROR: Occurence of unchanged object but having "
                     << " multiple correspondents. "
                     << std::endl;
           exit (EXIT_FAILURE);
        }
    }
    else if (change_status == -2)
    {
        // The geometry of the building has changed
        if(correspondence_length == 2)
        {
            // The original building has either been modified or merged (with other
            // buildings) into the target building
            int new_index = change.Compare->first[building_index][1];
            std::string target_id = to_global_id(time_stamp2,
                                               (*change.BuildingID2)[new_index]);
            iter = ReverseNodeIndex.find(target_id);
            if (iter == ReverseNodeIndex.end())
            {
                std::cout << "Unfound target id within ReverseNodeIndex: "
                          << target_id
                          << std::endl;
                exit (EXIT_FAILURE);
            }
            int target_node_id = iter->second;

            pt::ptree edge;
            edge.put("id", boost::lexical_cast<std::string>(edge_index++));
            edge.put("source", source_node_id);
            edge.put("target", target_node_id);

            // Check the length of the correspondence of the target building
            // to find out if the original building has been modified (1->1 relationship)
            // or merged with others into the target building
            auto reverse_correspondence = change.Compare->second[new_index];
            std::size_t reverse_correspondence_length = reverse_correspondence.size();

            if(reverse_correspondence_length == 2) {
                // The building geometry has changed
                // e.g. it was heightened; its footprint geometry changed, etc.
                edge.put("comment", "Modification: Modified geometry, different ID");
            }
            else {
                // The target building has more than one correspondent in the original buildings
                // list (i.e. the original building has been merged with others in the target building)
                // Note that the other nodes merged with the current original building into the target
                // building will be managed later in this building for loop
                edge.put("comment", "Fusion: Merged (with others) into target");
            }
            edges.push_back(std::make_pair("", edge));
            continue;
        }

        // We have more than one building correspondent. The original building was
        // thus either
        //   * split in sub-parts (while preserving its outside geometry)
        //   * revamped to new buildings (with a new total footprint included
        //     in the footprint of the original building): ASSERT THIS!

        for (std::size_t j = 0; j < correspondence_length; j += 2)
        {
          int change_status =  change.Compare->first[building_index][j];
          int new_index     =  change.Compare->first[building_index][j+1];
          std::string target_id = to_global_id(time_stamp2,
                                               (*change.BuildingID2)[new_index]);
          iter = ReverseNodeIndex.find(target_id);
          if (iter == ReverseNodeIndex.end())
          {
            std::cout << "Unfound target id within ReverseNodeIndex: "
                      << target_id
                      << std::endl;
            exit (EXIT_FAILURE);
          }
          int target_node_id = iter->second;

          // The building has many correspondent building and hence was subdivided
          // in many buldings: add an edge for each corresponding building:
          pt::ptree edge;
          edge.put("id", boost::lexical_cast<std::string>(edge_index++));
          edge.put("source", source_node_id);
          edge.put("target", target_node_id);
          edge.put("comment", "Subdivided into (many) buildings");
          edges.push_back(std::make_pair("", edge));
        }  // for on correspondence_length
    }
    else
    {
        std::cout << "ERRONEOUS change status: must be -1 or -2."
                  << std::endl;
        exit (EXIT_FAILURE);
    }
  }  // For on buildings

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
