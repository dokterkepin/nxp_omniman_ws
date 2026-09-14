# Who controls the robot right now. Published latched on /control/owner by
# control_arbiter, so a node starting up learns the current owner at once.

# Name of the owner, e.g. "nav" or "policy". Empty when nobody holds control.
string owner

# Fully qualified name of the node that acquired control, e.g. "/policy_runner".
# control_arbiter releases control on its own if this node disappears.
# Empty when nobody holds control, or when the holder gave no node name.
string node

# When the current owner acquired control.
builtin_interfaces/Time since
