#! /bin/env python3
import sys
import re
import collections
import textwrap

#To generate the .dot file:
    #$ ./extract_state_graph.py ihm.c > graph.dot

#To generate the png file:
    #$ dot -Tpng graph.dot > output.png



state_regex = re.compile('.*STATE_(\w+).*')
state_decl_regex = re.compile('.*case.*');
inline_comment_regex = re.compile('.*?//(.*)');
graphviz_comment_regex = re.compile('.*?//STATE_DIAGRAM:(.*)');


origin_state = None
state_dict = collections.defaultdict(list)
state_set = set()

with open(sys.argv[1]) as ihm_file:
    graph_custom_commands = ""

    for line in ihm_file:
        match = state_regex.match(line)
        line = line.strip()

        graphviz_comment_match = graphviz_comment_regex.match(line)
        if graphviz_comment_match:
            graph_custom_commands += graphviz_comment_match.group(1)+'\n'
            continue


        if line == 'default:':
            origin_state = "Default"

        if match and not line.startswith('//'):
            current_state = match.group(1).strip()
            state_set.add(current_state)

            if state_decl_regex.match(line):
                origin_state = current_state
            else:
                if origin_state is None:
                    continue

                comment = ""
                comment_match = inline_comment_regex.match(line)
                if comment_match:
                    comment = comment_match.group(1).strip()
                comment = textwrap.fill(comment, 30);

                #if not current_state in state_dict[origin_state]:
                state_dict[origin_state].append((current_state,comment))

    graph = "digraph finite_state_machine {\n"
    graph += " ".join(state_set)+";\n"
    for state, child_list in state_dict.items():
        for child in child_list:
            graph += state+" -> "+child[0]+' [label="'+child[1]+'"];\n'

    graph += '\n'+graph_custom_commands
    graph += "\n}"
    print(graph)
