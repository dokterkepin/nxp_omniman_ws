# Custom Behavior Tree Nodes

This directory contains copy-and-edit templates for user-defined BT nodes.
They live outside `actions/` and `controls/` so they do not appear in BT
Manager until a user copies one into the runtime node folders.

## Add An Action

1. Copy `action_template.py` to
   `orchestrator/orchestrator/bt/actions/my_action.py`.
2. Rename the class. The class name becomes the XML tag shown in BT Manager.
3. Implement `tick()` and return `NodeStatus.RUNNING`, `SUCCESS`, or `FAILURE`.
4. Click **Refresh Nodes** in BT Manager.

For simple constructor parameters, no extra loader code is needed. The generic
loader passes XML attributes into matching `__init__` kwargs.

Use `from_xml_params(context, name, params)` when the action needs runtime
dependencies such as the ROS node, topic config, joint names, or helper methods.
Most user actions do not need this method.

## Add A Control

1. Copy `control_template.py` to
   `orchestrator/orchestrator/bt/controls/my_control.py`.
2. Rename the class. The class name becomes the XML tag shown in BT Manager.
3. Implement how children are ticked.
4. Implement `get_active_node_ids()` when the UI should highlight the active
   child while the tree is running.
5. Click **Refresh Nodes**.

## Discovery Rules

- New files are discovered by scanning the `actions/` and `controls/` folders.
- Editing `__init__.py` is not required for BT Manager or XML execution.
- `__init__.py` is only for package-level imports such as
  `from orchestrator.bt.actions import Wait`.
- Constructor kwargs become ports; type hints and defaults become port types
  and default values.
- If running from an installed package instead of a source-mounted workspace,
  rebuild/restart before refreshing so the new file exists in the install space.
