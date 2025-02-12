#!/usr/bin/env python
import re
import sys
import os

def collect_comp_docs(infile):
    description = None
    with open(infile, 'r') as f:
        content = f.read()
        # Extract comments starting with #
        comment_pattern = re.compile(r'#(.*)', re.DOTALL)
        comments = comment_pattern.findall(content)

        if comments:
            # Join consecutive comments to form the description and remove leading hashtags
            description_lines = []
            for comment in comments:
                lines = comment.split('\n')
                cleaned_lines = [re.sub(r'^#\s*', '', line).strip() for line in lines]
                description_lines.extend(cleaned_lines)
            description = '\n'.join(description_lines).strip()

    compname = os.path.basename(infile).split('.')[0].upper()
    formatted_description = f"# {compname}\n{description}"
    return formatted_description

def extract_conf_content(infile):
    with open(infile, 'r') as f:
        content = f.readlines()
    # Extract the configuration content
    conf_content = '\n'.join(content)
    return conf_content

def main(output_file, config_templates):
    # Find which template file has the same name as the output file, ignoring extensions
    matching_template_file = None
    for template_file in config_templates:
        if os.path.splitext(os.path.basename(template_file))[0] == os.path.splitext(os.path.basename(output_file))[0]:
            matching_template_file = template_file
            break
        
    # Collect component documentation and write to Markdown file
    description = collect_comp_docs(matching_template_file)

    conf_content = ""
    if matching_template_file:
        conf_content = extract_conf_content(matching_template_file)

    with open(output_file, 'w') as md_file:
        md_file.write(description + '\n\n')
        if conf_content:
            md_file.write("## Configuration Content\n")
            md_file.write("```ini\n")
            md_file.write(conf_content)
            md_file.write("\n```\n")

        # Add the link to the configuration file
        conf_link = f"Source: [Configuration File](https://github.com/freakontrol/stmbl/blob/main/{matching_template_file})"

        # Add the graph image
        output_base_name = os.path.splitext(os.path.basename(output_file))[0]
        graph_image_path = f"/graph/f4_template_{output_base_name}.dot.svg"
        graph_image_markdown = f'\n\n{{{{< zoomable-image src="{graph_image_path}" alt="Description of image" >}}}}\n'
        md_file.write(graph_image_markdown)
        md_file.write(f"\n\n{conf_link}\n")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: create_template_doc.py <output_file> <config_templates...>")
        sys.exit(1)

    output_file = sys.argv[1]
    config_templates = sys.argv[2:]  # Extract configuration templates

    main(output_file, config_templates)