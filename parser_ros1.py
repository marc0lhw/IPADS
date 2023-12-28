import subprocess
import json
import re
from datetime import datetime

current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

def run_command(command):
	result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
	return result.stdout.strip().decode()

def get_topic_list():
	command = "rostopic list"
	topics = run_command(command).split('\n')
	return topics

def get_topic_info(topic):
    command = f"rostopic info {topic}"
    info = run_command(command)
    return info

def parse_topic_info(info, topic_name):
    paragraphs = info.split('\n\n')
    publishers = []
    subscribers = []

    for paragraph in paragraphs:
        if paragraph.startswith("Type:"):
            continue
        elif paragraph.startswith("Publishers:"):
            lines = paragraph.split('\n')

            if(len(lines[0].split()) == 2):
                publishers = None
            else:
                publishers = parse_publishers_subscribers(lines[1:])
        elif paragraph.startswith("Subscribers:"):
            lines = paragraph.split('\n')

            if(len(lines[0].split()) == 2):
                subscribers = None
            else:
                subscribers = parse_publishers_subscribers(lines[1:])

    return {
        "topic_name": topic_name,
        "pub": publishers,
        "sub": subscribers
    }

def parse_publishers_subscribers(lines):
    result = []

    for element in lines:
        element = element.strip()
        parts = element.split()
        node_name = parts[1]
        ip_port = parts[2].strip('(http://)')
        ip, port = ip_port.split(':')
        result.append({"node_name": node_name, "ip": ip, "port": port})
    return result

def topic_parser():
    topics = get_topic_list()
    topic_data = {"topics": []}

    output_file = f"ros1_topics_{current_time}.json"

    for topic in topics:
        info = get_topic_info(topic)
        parsed_info = parse_topic_info(info, topic)
        topic_data["topics"].append(parsed_info)

    with open(output_file, "w") as json_file:
        json.dump(topic_data, json_file, indent=4)

def get_rosnode_list():
    result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.splitlines()
    else:
        return []

def get_rosnode_info(node_name):
    result = subprocess.run(['rosnode', 'info', node_name], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout
    else:
        return None

def parse_rosnode_info(info_output):
    parsed_info = {
        'node_name': '',
        'Publications': [],
        'Subscriptions': [],
        'Services': []
    }

    section = None

    for line in info_output.splitlines():
        if line.startswith('Node ['):
            parsed_info['node_name'] = line.split('[', 1)[1].split(']')[0]

        if line.startswith('contacting node'):
            break  # 파싱 중단

        if line.startswith('Publications:'):
            section = 'Publications'
        elif line.startswith('Subscriptions:'):
            section = 'Subscriptions'
        elif line.startswith('Services:'):
            section = 'Services'
        elif line.startswith(' * '):
            # 새로운 패턴 추가
            match = re.match(r'^\s+\*\s+(.+)\s+\[(.+)\]$', line)
            if match:	# pub & sub
                topic_name, topic_type = match.groups()
                parsed_info[section].append({'topic_name': topic_name, 'topic_type': topic_type})
            else:	# services
                parsed_info[section].append(line.strip(' *'))


    # 모든 섹션이 비어있으면 해당 섹션은 None으로 설정
    for key, value in parsed_info.items():
        if key != 'node_name' and not value:
            parsed_info[key] = None

    return parsed_info

def node_parser():
    nodes = get_rosnode_list()
    node_info_list = []

    for node_name in nodes:
        info_output = get_rosnode_info(node_name)
        if info_output is not None:
            parsed_info = parse_rosnode_info(info_output)
            node_info_list.append(parsed_info)

    data_to_save = {'nodes': node_info_list}

    output_file = f"ros1_nodes_{current_time}.json"

    with open(output_file, 'w') as json_file:
        json.dump(data_to_save, json_file, indent=4)

def get_rosservice_list():
    result = subprocess.run(['rosservice', 'list'], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.splitlines()
    else:
        return []

def get_rosservice_info(service_name):
    result = subprocess.run(['rosservice', 'info', service_name], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout
    else:
        return None

def parse_rosservice_info(info_output, service_name):
    parsed_info = {
        'service_name': '',
        'node': '',
        'uri_ip': '',
        'uri_port': '',
        'type': '',
        'args': None
    }

    for line in info_output.splitlines():
        if line.startswith('Node:'):
            parsed_info['node'] = line.split(': ', 1)[1]
        elif line.startswith('URI:'):
            uri_match = re.match(r'rosrpc://(\S+):(\d+)', line.split(': ', 1)[1])
            if uri_match:
                parsed_info['uri_ip'], parsed_info['uri_port'] = uri_match.groups()
        elif line.startswith('Type:'):
            parsed_info['type'] = line.split(': ', 1)[1]
        elif line.startswith('Args:'):
            args_match = re.match(r'(.+)$', line.split(': ', 1)[1])
            if args_match:
                parsed_info['args'] = args_match.groups()

    parsed_info['service_name'] = service_name 
    
    return parsed_info

def service_parser():
    services = get_rosservice_list()
    service_info_list = []

    for service_name in services:
        info_output = get_rosservice_info(service_name)
        if info_output is not None:
            parsed_info = parse_rosservice_info(info_output, service_name)
            service_info_list.append(parsed_info)

    data_to_save = {'services': service_info_list}

    output_file = f"ros1_services_{current_time}.json"

    with open(output_file, 'w') as json_file:
        json.dump(data_to_save, json_file, indent=4)

if __name__ == "__main__":
    topic_parser()
    node_parser()
    service_parser()
