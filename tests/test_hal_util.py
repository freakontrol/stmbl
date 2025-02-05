import unittest
from docs.tools.hal_graph.hal_util import (
    remove_commented_lines,
    parse_commands,
    parse_config_commands,
    collect_components,
    collect_template_conf
)

#TODO: complete the tests with temporary files
class TestHalUtil(unittest.TestCase):
    pass
#     def test_remove_commented_lines(self):
#         content = "// This is a comment\nline1\n# This is another comment\nline2"
#         expected_output = "line1\nline2"
#         self.assertEqual(remove_commented_lines(content), expected_output)

#     def test_parse_commands(self):
#         content = '''hal_parse("command1")
# // hal_parse("commented_command")
# hal_parse("command2")'''
#         expected_output = ["command1", "command2"]
#         self.assertEqual(parse_commands(content), expected_output)

    def test_parse_config_commands(self):
        pass
#         content = """config_command1
# # config_command2
# config_command3"""
#         expected_output = ["config_command1", "# config_command2", "config_command3"]
#         self.assertEqual(parse_config_commands(content), expected_output)

    def test_collect_components(self):
        pass

    def test_collect_template_conf(self):
        pass

if __name__ == '__main__':
    unittest.main()