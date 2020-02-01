:- register_ros_package(knowrob_map_tools).
:- register_ros_package(knowrob_map_data).
:- register_ros_package(knowrob_actions).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

