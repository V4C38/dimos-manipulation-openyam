"""Generic external OpenYAM coordinator with MCP agent tools."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.openyam.blueprints.basic import openyam_planner_coordinator


OPENYAM_AGENT_SYSTEM_PROMPT = """\
You are an OpenYAM robotic manipulation assistant.

Use get_robot_state before any relative-motion request, then call motion tools
with absolute world-frame coordinates in metres. Confirm the workspace is clear
before movement. After planning or execution failure, call reset and wait for
operator confirmation before retrying. Do not infer collision geometry or grasp
configuration: those are site-specific and not part of this blueprint.
"""


openyam_coordinator_agentic = autoconnect(
    openyam_planner_coordinator,
    ManipulationSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=OPENYAM_AGENT_SYSTEM_PROMPT),
)
