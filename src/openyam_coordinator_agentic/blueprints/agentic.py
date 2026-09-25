"""OpenYAM planner and coordinator with an MCP manipulation agent."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.robot.manipulators.openyam.blueprints.basic import openyam_planner_coordinator
from openyam_coordinator_agentic.agent_prompts import OPENYAM_AGENT_SYSTEM_PROMPT

openyam_planner_coordinator_agent = autoconnect(
    openyam_planner_coordinator,
    ManipulationSkills.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=OPENYAM_AGENT_SYSTEM_PROMPT),
)
