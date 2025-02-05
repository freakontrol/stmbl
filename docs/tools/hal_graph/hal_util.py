import re
import os

def remove_commented_lines(content):
    # Remove lines that are comments or contain comments
    lines = content.split('\n')
    non_comment_lines = [line for line in lines if not line.strip().startswith('//')]
    non_comment_lines = [line for line in lines if not line.strip().startswith('#')]
    return '\n'.join(non_comment_lines)

def parse_commands(content):
    # Remove commented lines
    cleaned_content = remove_commented_lines(content)
    # Extract commands from hal_parse() function calls
    command_pattern = re.compile(r'hal_parse\("([^"]+)"\)')
    commands = command_pattern.findall(cleaned_content)
    return commands

def parse_config_commands(content):
    # Remove commented lines
    cleaned_content = remove_commented_lines(content)
    commands = cleaned_content.split('\n')
    return commands

def collect_components(comp_files):

    comps = {}

    for infile in comp_files:
        with open(infile) as f:
            pins = []
            compname = ''
            for line in f:
                comp = re.search('COMP\((\w*)\);', line)
                if comp:
                    compname = comp.groups()[0]
                    comps[compname] = []
                pin = re.search('HAL_PIN\((\w*)\)', line)
                if pin:
                    pins.append(pin.groups()[0])
                pin = re.search('HAL_PINA\((\w*),\s*(\d*)\)', line)
                if pin:
                    for i in range(int(pin.groups()[1])):
                        pins.append(f"{pin.groups()[0]}{i}")
            pins.append("rt_prio")
            pins.append("frt_prio")
            comps[compname].extend(pins)
            
    return comps

def collect_template_conf(config_templates):
    
    config_commands = {}
    
    for template_file in config_templates:
        with open(template_file, 'r') as f:
            content = f.read()
            commands = parse_config_commands(content)
            config_name = os.path.basename(template_file).split('.')[0]
            config_commands[config_name] = commands
            
    return config_commands