#! /bin/env python3
import sys
import re
import collections

#To generate the .dot file:
    #$ ./extract_state_graph.py ihm.c > graph.dot

#To generate the png file:
    #$ dot -Tpng graph.dot > output.png



state_regex = re.compile('.*STATE_(\w+).*')
state_decl_regex = re.compile('.*case.*');

origin_state = None
state_dict = collections.defaultdict(list)
state_set = set()

with open(sys.argv[1]) as ihm_file:
    for line in ihm_file:
        match = state_regex.match(line)
        if match:
            current_state = match.group(1)
            state_set.add(current_state)

            if state_decl_regex.match(line):
                origin_state = current_state
            else:
                if origin_state is None:
                    continue
                if not current_state in state_dict[origin_state]:
                    state_dict[origin_state].append(current_state)

    graph = "digraph finite_state_machine {\n"
    graph += " ".join(state_set)+";\n"
    for state, child_list in state_dict.items():
        for child in child_list:
            graph += state+" -> "+child+" [label="+'""'+"];\n"
    graph += "}"
    print(graph)
    #for

    #print(match)
    #print(state_dict)
    #print(line)
