import os
import sys
from collections import OrderedDict
try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import StringIO  # Python 3.x

# ROS 1 imports
import ament_index_python
import genmsg.msg_loader
from genmsg.base import COMMENTCHAR, IODELIM
import rosmsg
import rospkg
from catkin_pkg.package import parse_package
import yaml
from init import Message, Mapping, MappingRule
# from rosidl_cmake import expand_template
import rosidl_adapter.parser
from rosidl_cmake import expand_template


# genmsg/actions.py?
class ActionSpec(object):

    def __init__(self, goal, result, feedback, text, full_name='', short_name='', package=''):

        alt_package, alt_short_name = genmsg.package_resource_name(full_name)
        if not package:
            package = alt_package
        if not short_name:
            short_name = alt_short_name

        self.goal = goal
        self.result = result
        self.feedback = feedback
        self.text = text
        self.full_name = full_name
        self.short_name = short_name
        self.package = package

    def __eq__(self, other):
        if not other or not isinstance(other, ActionSpec):
            return False
        return self.goal == other.goal and \
            self.result == other.result and \
            self.feedback == other.feedback and \
            self.text == other.text and \
            self.full_name == other.full_name and \
            self.short_name == other.short_name and \
            self.package == other.package

    def __ne__(self, other):
        if not other or not isinstance(other, ActionSpec):
            return True
        return not self.__eq__(other)

    def __repr__(self):
        return "ActionSpec[%s, %s, %s]" % (repr(self.goal), repr(self.result), repr(self.feedback))


# genmsg.msg_loader
def load_action_from_string(msg_context, text, full_name):
    """
    Load :class:`ActionSpec` from the .action file.

    :param msg_context: :class:`MsgContext` instance to load goal/reresult/feedback messages into.
    :param text: .msg text , ``str``
    :param package_name: context to use for msg type name, i.e. the package name,
      or '' to use local naming convention. ``str``
    :returns: :class:`ActionSpec` instance
    :raises :exc:`InvalidMsgSpec` If syntax errors or other problems are detected in file
    """
    text_goal = StringIO()
    text_result = StringIO()
    text_feedback = StringIO()
    count = 0
    accum = text_goal
    for l in text.split('\n'):
        l = l.split(COMMENTCHAR)[0].strip()  # strip comments
        if l.startswith(IODELIM):  # lenient, by request
            if count == 0:
                accum = text_result
                count = 1
            else:
                accum = text_feedback
        else:
            accum.write(l+'\n')

    # create separate MsgSpec objects for each half of file
    msg_goal = genmsg.msg_loader.load_msg_from_string(
        msg_context, text_goal.getvalue(), '%sGoal' % (full_name))
    msg_result = genmsg.msg_loader.load_msg_from_string(
        msg_context, text_result.getvalue(), '%sResult' % (full_name))
    msg_feedback = genmsg.msg_loader.load_msg_from_string(
        msg_context, text_feedback.getvalue(), '%sFeedback' % (full_name))
    return ActionSpec(msg_goal, msg_result, msg_feedback, text, full_name)


# genmsg.msg_loader
def load_action_from_file(msg_context, file_path, full_name):
    """
    Convert the .action representation in the file to a :class:`MsgSpec` instance.
    NOTE: this will register the message in the *msg_context*.

    :param file_path: path of file to load from, ``str``
    :returns: :class:`MsgSpec` instance
    :raises: :exc:`InvalidMsgSpec`: if syntax errors or other problems are detected in file
    """
    with open(file_path, 'r') as f:
        text = f.read()

    spec = load_action_from_string(msg_context, text, full_name)
    # msg_context.set_file('%sRequest' % (full_name), file_path)
    # msg_context.set_file('%sResponse' % (full_name), file_path)
    return spec


# __init__.py
def load_ros1_action(ros1_action):
    msg_context = genmsg.MsgContext.create_default()
    message_path = os.path.join(
        ros1_action.prefix_path, ros1_action.message_name + '.action')
    try:
        spec = load_action_from_file(
            msg_context, message_path, '%s/%s' % (ros1_action.package_name, ros1_action.message_name))
    except genmsg.InvalidMsgSpec:
        return None
    return spec


# __init__.py
def load_ros2_action(ros2_action):
    actiom_path = os.path.join(
        ros2_action.prefix_path, 'share', ros2_action.package_name, 'action',
        ros2_action.message_name + '.action')
    try:
        spec = rosidl_adapter.parser.parse_action_file(
            ros2_action.package_name, actiom_path)
    except rosidl_adapter.parser.InvalidSpecification:
        print("Invalid spec")
        return None
    return spec


# rosmsg.py
def iterate_action_packages(rospack):
    subdir = 'action'
    pkgs = rospack.list()
    for p in pkgs:
        package_paths = rosmsg._get_package_paths(p, rospack)
        for package_path in package_paths:
            d = os.path.join(package_path, subdir)
            if os.path.isdir(d):
                yield p, d


# __init__.py
def get_ros1_actions(rospack=None):
    if not rospack:
        rospack = rospkg.RosPack()
    actions = []
    # actual function: rosmsg.iterate_packages(rospack, rosmsg.MODE_MSG))
    pkgs = sorted(x for x in iterate_action_packages(rospack))
    for package_name, path in pkgs:
        for action_name in rosmsg._list_types(path, 'msg', ".action"):
            actions.append(Message(package_name, action_name, path))
    return actions


def get_ros2_actions():
    pkgs = []
    actions = []
    rules = []
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
        pkgs.append(package_name)
        resource, _ = ament_index_python.get_resource(
            resource_type, package_name)
        interfaces = resource.splitlines()
        action_names = {
            i[7:-7]
            for i in interfaces
            if i.startswith('action/') and i[-7:] in ('.idl', '.action')}
        for action_name in sorted(action_names):
            actions.append(Message(package_name, action_name, prefix_path))
        package_path = os.path.join(prefix_path, 'share', package_name)
        pkg = parse_package(package_path)
        for export in pkg.exports:
            if export.tagname != 'ros1_bridge':
                continue
            if 'mapping_rules' not in export.attributes:
                continue
            rule_file = os.path.join(
                package_path, export.attributes['mapping_rules'])
            with open(rule_file, 'r') as h:
                content = yaml.safe_load(h)
            if not isinstance(content, list):
                print(
                    "The content of the mapping rules in '%s' is not a list" % rule_file,
                    file=sys.stderr)
                continue
            for data in content:
                if all(n not in data for n in ('ros1_message_name', 'ros2_message_name', 'ros1_service_name', 'ros2_service_name')):
                    try:
                        rules.append(ActionMappingRule(data, package_name))
                    except Exception as e:
                        print('%s' % str(e), file=sys.stderr)
    return pkgs, actions, rules


class ActionMappingRule(MappingRule):
    __slots__ = [
        'ros1_action_name',
        'ros2_action_name',
        'goal_fields_1_to_2',
        'result_fields_1_to_2',
        'feedback_fields_1_to_2',
    ]

    def __init__(self, data, expected_package_name):
        super().__init__(data, expected_package_name)
        self.ros1_action_name = None
        self.ros2_action_name = None
        self.goal_fields_1_to_2 = None
        self.result_fields_1_to_2 = None
        self.feedback_fields_1_to_2 = None
        if all(n in data for n in ('ros1_action_name', 'ros2_action_name')):
            self.ros1_action_name = data['ros1_action_name']
            self.ros2_action_name = data['ros2_action_name']
            expected_keys = 4
            if 'goal_fields_1_to_2' in data:
                self.goal_fields_1_to_2 = OrderedDict()
                for ros1_field_name, ros2_field_name in data['goal_fields_1_to_2'].items():
                    self.goal_fields_1_to_2[ros1_field_name] = ros2_field_name
                expected_keys += 1
            if 'result_fields_1_to_2' in data:
                self.result_fields_1_to_2 = OrderedDict()
                for ros1_field_name, ros2_field_name in data['result_fields_1_to_2'].items():
                    self.result_fields_1_to_2[ros1_field_name] = ros2_field_name
                expected_keys += 1
            if 'feedback_fields_1_to_2' in data:
                self.feedback_fields_1_to_2 = OrderedDict()
                for ros1_field_name, ros2_field_name in data['feedback_fields_1_to_2'].items():
                    self.feedback_fields_1_to_2[ros1_field_name] = ros2_field_name
                expected_keys += 1
            elif len(data) > expected_keys:
                raise RuntimeError(
                    'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)
        elif len(data) > 2:
            raise RuntimeError(
                'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)

    def __str__(self):
        return 'ActionMappingRule(%s, %s)' % (self.ros1_package_name, self.ros2_package_name)


def determine_common_actions(
    ros1_actions, ros2_actions, mapping_rules, message_string_pairs=None
):
    if message_string_pairs is None:
        message_string_pairs = set()

    pairs = []
    actions = []
    for ros1_action in ros1_actions:
        for ros2_action in ros2_actions:
            if ros1_action.package_name == ros2_action.package_name:
                if ros1_action.message_name == ros2_action.message_name:
                    pairs.append((ros1_action, ros2_action))

    for rule in mapping_rules:
        for ros1_action in ros1_actions:
            for ros2_action in ros2_actions:
                if rule.ros1_package_name == ros1_action.package_name and \
                   rule.ros2_package_name == ros2_action.package_name:
                    if rule.ros1_service_name is None and rule.ros2_service_name is None:
                        if ros1_action.message_name == ros2_action.message_name:
                            pairs.append((ros1_action, ros2_action))
                    else:
                        if (
                            rule.ros1_service_name == ros1_action.message_name and
                            rule.ros2_service_name == ros2_action.message_name
                        ):
                            pairs.append((ros1_action, ros2_action))

    for pair in pairs:
        ros1_spec = load_ros1_action(pair[0])
        ros2_spec = load_ros2_action(pair[1])
        ros1_fields = {
            'goal': ros1_spec.goal.fields(),
            'result': ros1_spec.result.fields(),
            'feedback': ros1_spec.feedback.fields()
        }
        ros2_fields = {
            'goal': ros2_spec.goal.fields,
            'result': ros2_spec.result.fields,
            'feedback': ros2_spec.feedback.fields
        }
        output = {
            'goal': [],
            'result': [],
            'feedback': []
        }
        match = True
        for direction in ['goal', 'result', 'feedback']:
            if len(ros1_fields[direction]) != len(ros2_fields[direction]):
                match = False
                break
            for i, ros1_field in enumerate(ros1_fields[direction]):
                ros1_type = ros1_field[0]
                ros2_type = str(ros2_fields[direction][i].type)
                ros1_name = ros1_field[1]
                ros2_name = ros2_fields[direction][i].name
                if ros1_type != ros2_type or ros1_name != ros2_name:
                    # if the message types have a custom mapping their names
                    # might not be equal, therefore check the message pairs
                    # the check for 'builtin_interfaces' should be removed once merged with __init__.py
                    # It seems to handle it already
                    if (ros1_type, ros2_type) not in message_string_pairs and not ros2_type.startswith("builtin_interfaces"):
                        print("custom: " + ros1_type + " " + ros2_type +
                              " : " + ros1_name + " " + ros2_name)
                        match = False
                        break
                output[direction].append({
                    'basic': False if '/' in ros1_type else True,
                    'array': True if '[]' in ros1_type else False,
                    'ros1': {
                        'name': ros1_name,
                        'type': ros1_type.rstrip('[]'),
                        'cpptype': ros1_type.rstrip('[]').replace('/', '::')
                    },
                    'ros2': {
                        'name': ros2_name,
                        'type': ros2_type.rstrip('[]'),
                        'cpptype': ros2_type.rstrip('[]').replace('/', '::msg::')
                    }
                })
        if match:
            actions.append({
                'ros1_name': pair[0].message_name,
                'ros2_name': pair[1].message_name,
                'ros1_package': pair[0].package_name,
                'ros2_package': pair[1].package_name,
                'fields': output
            })
    return actions


# __init__.py
def generate_actions(rospack=None, message_string_pairs=None):
    # TODO: find all ROS1 and ROS2 actions
    ros1_actions = get_ros1_actions(rospack)
    ros2_pkgs, ros2_actions, mapping_rules = get_ros2_actions()

    actions = determine_common_actions(
        ros1_actions, ros2_actions, mapping_rules, message_string_pairs=message_string_pairs)
    return {
        'actions': actions,
        'ros2_package_names_actions': ros2_pkgs,
        'all_ros2_actions': ros2_actions,
    }


# __init__.py
def generate_cpp(output_path, template_dir):
    rospack = rospkg.RosPack()
    ros1_time = Message("std_msgs", "Time",
                        "/opt/ros/melodic/share/std_msgs/msg")
    ros1_dur = Message("std_msgs", "Duration",
                       "/opt/ros/melodic/share/std_msgs/msg")
    ros2_time = Message("builtin_interfaces", "Time",
                        "/opt/ros/dashing")
    ros2_dur = Message("builtin_interfaces", "Duration",
                       "/opt/ros/dashing")

    ros1_msgs = [ros1_time, ros1_dur]
    ros2_msgs = [ros2_time, ros2_dur]

    mappings = []
    # add custom mapping for builtin_interfaces
    for msg_name in ('Duration', 'Time'):
        ros1_msg = [
            m for m in ros1_msgs
            if m.package_name == 'std_msgs' and m.message_name == msg_name]
        print(ros1_msg)
        ros2_msg = [
            m for m in ros2_msgs
            if m.package_name == 'builtin_interfaces' and m.message_name == msg_name]
        print(ros2_msg)
        if ros1_msg and ros2_msg:
            # print(Mapping(ros1_msg[0], ros2_msg[0]))
            mappings.append(Mapping(ros1_msg[0], ros2_msg[0]))

    message_string_pairs = {
        (
            '%s/%s' % (m.ros1_msg.package_name, m.ros1_msg.message_name),
            '%s/%s' % (m.ros2_msg.package_name, m.ros2_msg.message_name))
        for m in mappings}
    print(message_string_pairs)

    data = generate_actions(rospack, message_string_pairs)

    unique_package_names = set(
        data['ros2_package_names_actions'])
    # skip builtin_interfaces since there is a custom implementation
    unique_package_names -= {'builtin_interfaces'}
    data['ros2_package_names'] = list(unique_package_names)

    for ros2_package_name in data['ros2_package_names']:
        for interface_type, interfaces in zip(
            ['action'], [data['all_ros2_actions']]
        ):
            for interface in interfaces:
                if interface.package_name != ros2_package_name:
                    continue
                data_idl_cpp = {
                    'ros2_package_name': ros2_package_name,
                    'interface_type': interface_type,
                    'interface': interface,
                    'mapped_actions': [
                        s for s in data['actions']
                        if s['ros2_package'] == ros2_package_name and
                        s['ros2_name'] == interface.message_name],
                }
                print(data_idl_cpp['mapped_actions'])
                template_file = os.path.join(
                    template_dir, 'interface_factories.cpp.em')
                output_file = os.path.join(
                    output_path, '%s__%s__%s__factories.cpp' %
                    (ros2_package_name, interface_type, interface.message_name))
                print(output_file)
                output_file = "generated/" + output_file
                if not os.path.exists(output_file):
                    open(output_file, 'w').close()
                expand_template(template_file, data_idl_cpp, output_file)


if __name__ == "__main__":
    generate_cpp("", "")
