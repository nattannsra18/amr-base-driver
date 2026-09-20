"""Credential rotation tests for the Robot Agent."""

import asyncio

from amr_web_bridge.agent_identity import AgentCredentialStore
from amr_web_bridge.web_bridge_node import WebBridgeNode


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


class _RotationAgent:
    handle_credential_rotation = WebBridgeNode.handle_credential_rotation
    send_agent_hello = WebBridgeNode.send_agent_hello

    def __init__(self, credential_path):
        self.robot_id = 'robot-1'
        self.robot_credential = 'old-secret'
        self.robot_credential_version = 1
        self.agent_boot_id = 'boot-1'
        self.agent_version = '0.2.0'
        self.profile_version = 'test-v1'
        self.agent_capabilities = ['navigation']
        self.robot_serial_number = 'SIM-0001'
        self.hardware_fingerprint = 'derived-hardware-fingerprint'
        self.credential_store = AgentCredentialStore(credential_path)
        self.sent = []
        self.logger = _Logger()

    async def send_json(self, _websocket, message):
        self.sent.append(message)

    def get_logger(self):
        return self.logger


def test_rotation_is_saved_before_agent_acknowledges(tmp_path):
    agent = _RotationAgent(tmp_path / 'credential.json')
    asyncio.run(agent.handle_credential_rotation(object(), {
        'type': 'credential_rotation',
        'protocol_version': '1.0',
        'robot_id': 'robot-1',
        'credential': 'replacement-secret-value-with-safe-length',
        'credential_version': 2,
    }))

    stored = agent.credential_store.load()
    assert stored is not None
    assert stored.credential_version == 2
    assert stored.credential == 'replacement-secret-value-with-safe-length'
    assert agent.robot_credential_version == 2
    assert agent.sent == [{
        'type': 'credential_rotated',
        'protocol_version': '1.0',
        'robot_id': 'robot-1',
        'credential_version': 2,
        'timestamp': agent.sent[0]['timestamp'],
    }]


def test_rotation_rejects_wrong_robot_without_changing_credential(tmp_path):
    agent = _RotationAgent(tmp_path / 'credential.json')
    asyncio.run(agent.handle_credential_rotation(object(), {
        'type': 'credential_rotation',
        'protocol_version': '1.0',
        'robot_id': 'robot-2',
        'credential': 'replacement-secret-value-with-safe-length',
        'credential_version': 2,
    }))

    assert agent.credential_store.load() is None
    assert agent.robot_credential == 'old-secret'
    assert agent.sent[0]['type'] == 'credential_rotation_failed'


def test_agent_hello_reports_paired_hardware_identity(tmp_path):
    agent = _RotationAgent(tmp_path / 'credential.json')
    asyncio.run(agent.send_agent_hello(object()))

    hello = agent.sent[0]
    assert hello['serial_number'] == 'SIM-0001'
    assert hello['hardware_fingerprint'] == 'derived-hardware-fingerprint'
