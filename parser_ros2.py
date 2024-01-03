import subprocess
import json
from datetime import datetime

current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

def parse_ros2_node_info(node_name):
    result = subprocess.run(['ros2', 'node', 'info', node_name], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout, result.stderr
    else:
        return None

def extract_topics_info(parsed_info, topics_info):
    if parsed_info['publications']:
        for pub in parsed_info['publications']:
            topic_name = pub['name']
            topic_type = pub['type']
            add_topic_info(topics_info, topic_name, topic_type, parsed_info['node_name'], None)

    if parsed_info['subscriptions']:
        for sub in parsed_info['subscriptions']:
            topic_name = sub['name']
            topic_type = sub['type']
            add_topic_info(topics_info, topic_name, topic_type, None, parsed_info['node_name'])

def extract_service_info(parsed_info, info_dict):
    if parsed_info['service_servers']:
        for entry in parsed_info['service_servers']:
            name = entry['name']
            server = entry['name'].split('/')[1] if entry['name'] else None
            client = None
            msg_type = entry['type']

            info_dict[name] = {
                'server': server,
                'client': client,
                'type': msg_type
            }

    if parsed_info['service_clients']:
        for entry in parsed_info['service_clients']:
            name = entry['name']
            server = None
            client = entry['name'].split('/')[1] if entry['name'] else None
            msg_type = entry['type']

            info_dict[name] = {
                'server': server,
                'client': client,
                'type': msg_type
            }

def extract_action_info(parsed_info, info_dict):
    if parsed_info['action_servers']:
        for entry in parsed_info['action_servers']:
            name = entry['name']
            server = entry['name'].split('/')[1] if entry['name'] else None
            client = None
            msg_type = entry['type']

            info_dict[name] = {
                'server': server,
                'client': client,
                'type': msg_type
            }

    if parsed_info['action_clients']:
        for entry in parsed_info['action_clients']:
            name = entry['name']
            server = None
            client = entry['name'].split('/')[1] if entry['name'] else None
            msg_type = entry['type']

            info_dict[name] = {
                'server': server,
                'client': client,
                'type': msg_type
            }

def add_topic_info(topics_info, topic_name, topic_type, pub_node, sub_node):
    if topic_name not in topics_info:
        topics_info[topic_name] = {
            'pub': [],
            'sub': [],
            'type': topic_type
        }

    if pub_node:
        topics_info[topic_name]['pub'].append(pub_node)

    if sub_node:
        topics_info[topic_name]['sub'].append(sub_node)

def save_ros2_additional_info_to_json(nodes_info, duplicate_nodes, warning_messages):
    topics_info = {}
    services_info = {}
    actions_info = {}

    for node_info in nodes_info:
        # Process topics
        extract_topics_info(node_info, topics_info)

        # Process services
        extract_service_info(node_info, services_info)

        # Process actions
        extract_action_info(node_info, actions_info)

    # Save topics info
    save_topic_info_to_json(topics_info)

    # Save services info
    save_service_info_to_json(services_info)

    # Save actions info
    save_action_info_to_json(actions_info)

def save_topic_info_to_json(topics_info):
    output_file = f"ros2_topics_{current_time}.json"
    with open(output_file, 'w') as json_file:
        data_to_save = {"topics": []}
        for topic_name, value in topics_info.items():
            entry = {
                "topic_name": topic_name,
                "pub": value.get("pub", None) if value.get("pub") else None,
                "sub": value.get("sub", None) if value.get("sub") else None,
                "type": value["type"]
            }
            data_to_save["topics"].append(entry)

        json.dump(data_to_save, json_file, indent=4)

def save_service_info_to_json(services_info):
    output_file = f"ros2_services_{current_time}.json"
    with open(output_file, 'w') as json_file:
        data_to_save = {"services": []}
        for service_name, value in services_info.items():
            entry = {
                "service_name": service_name,
                "server": value.get("server", None) if value.get("server") else None,
                "client": value.get("client", None) if value.get("client") else None,
                "type": value["type"]
            }
            data_to_save["services"].append(entry)

        json.dump(data_to_save, json_file, indent=4)

def save_action_info_to_json(actions_info):
    output_file = f"ros2_actions_{current_time}.json"
    with open(output_file, 'w') as json_file:
        data_to_save = {"actions": []}
        for action_name, value in actions_info.items():
            entry = {
                "action_name": action_name,
                "server": value.get("server", None) if value.get("server") else None,
                "client": value.get("client", None) if value.get("client") else None,
                "type": value["type"]
            }
            data_to_save["actions"].append(entry)

        json.dump(data_to_save, json_file, indent=4)

def save_ros2_node_info_to_json():
    result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
    if result.returncode != 0:
        print("Error getting list of nodes.")
        return

    nodes = result.stdout.splitlines()
    warnings = result.stderr.splitlines()
    node_info_list = []
    duplicate_nodes = {}
    warning_messages = []
    info_messages = {}

    # Check for warning messages
    warning_prefix = 'WARNING: '
    for warning in warnings:
        if warning.startswith(warning_prefix):
            warning_messages.append(warning)
    
    for node_name in nodes:
        info_output, info_err = parse_ros2_node_info(node_name)
        if info_output is not None:
            parsed_info = {
                'node_name': node_name,
                'subscriptions': [],
                'publications': [],
                'service_servers': [],
                'service_clients': [],
                'action_servers': [],
                'action_clients': [],
                'duplicated': "nodes in the graph with the exact name" in info_err,
                'info': info_err if info_err != "" else None
            }

            section = None

            for line in info_output.splitlines():
                if line.startswith('  Subscribers:'):
                    section = 'subscriptions'
                elif line.startswith('  Publishers:'):
                    section = 'publications'
                elif line.startswith('  Service Servers:'):
                    section = 'service_servers'
                elif line.startswith('  Service Clients:'):
                    section = 'service_clients'
                elif line.startswith('  Action Servers:'):
                    section = 'action_servers'
                elif line.startswith('  Action Clients:'):
                    section = 'action_clients'
                elif line.startswith('    '):
                    name, msg_type = line.strip().split(': ')
                    parsed_info[section].append({'name': name, 'type': msg_type})

             # Convert empty lists to null
            for key, value in parsed_info.items():
                if isinstance(value, list) and not value:
                    parsed_info[key] = None

            # if duplicated node
            if parsed_info['duplicated']:
                # Check for duplicate nodes
                if node_name in duplicate_nodes:
                    duplicate_nodes[node_name] += 1
                else:
                    duplicate_nodes[node_name] = 1

            # if info
            if parsed_info['info']:
                info_messages[node_name] = info_err

            node_info_list.append(parsed_info)

     # Save duplicate nodes and warning messages
    data_to_save = {
        'nodes': node_info_list,
        'duplicated_nodes': duplicate_nodes if duplicate_nodes else None,
        'warning_messages': warning_messages if warning_messages else None,
        'info_messages': info_messages if info_messages else None        
    }

    output_file = f"ros2_nodes_{current_time}.json"

    with open(output_file, 'w') as json_file:
        json.dump(data_to_save, json_file, indent=4)

    # additional info
    save_ros2_additional_info_to_json(node_info_list, duplicate_nodes, warning_messages)

if __name__ == "__main__":
    save_ros2_node_info_to_json()

