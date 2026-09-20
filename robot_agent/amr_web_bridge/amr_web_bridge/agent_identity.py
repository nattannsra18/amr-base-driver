"""Persistent identity and secure enrollment for a Robot Agent."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class EnrollmentError(RuntimeError):
    """An enrollment request failed with a normalized HTTP status."""

    def __init__(
        self,
        status: int,
        detail: str,
        retry_after_seconds: int | None = None,
    ):
        super().__init__(detail)
        self.status = status
        self.detail = detail
        self.retry_after_seconds = retry_after_seconds


@dataclass(frozen=True)
class AgentCredential:
    robot_id: str
    credential: str
    credential_version: int
    protocol_version: str = '1.0'


@dataclass(frozen=True)
class EnrollmentRequest:
    enrollment_id: str
    pairing_code: str
    expires_at: str
    poll_after_seconds: int


class AgentCredentialStore:
    """Stores the one-time credential in an owner-readable local file."""

    def __init__(self, path: Path):
        self.path = path

    def load(self) -> AgentCredential | None:
        try:
            payload = json.loads(self.path.read_text(encoding='utf-8'))
            credential = AgentCredential(
                robot_id=str(payload['robot_id']),
                credential=str(payload['credential']),
                credential_version=int(payload['credential_version']),
                protocol_version=str(payload.get('protocol_version', '1.0')),
            )
        except (FileNotFoundError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None
        if not credential.robot_id or not credential.credential:
            return None
        return credential

    def save(self, credential: AgentCredential) -> None:
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        temporary = self.path.with_name(f'.{self.path.name}.tmp')
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_TRUNC,
            0o600,
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(
                    {
                        'robot_id': credential.robot_id,
                        'credential': credential.credential,
                        'credential_version': credential.credential_version,
                        'protocol_version': credential.protocol_version,
                    },
                    stream,
                    separators=(',', ':'),
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            os.chmod(self.path, 0o600)
        finally:
            if temporary.exists():
                temporary.unlink()


class EnrollmentClient:
    """Minimal blocking HTTP client, called from an asyncio worker thread."""

    def __init__(self, server_url: str, bootstrap_token: str, timeout: float = 5.0):
        self.base_url = self._http_url(server_url)
        self.bootstrap_token = bootstrap_token
        self.timeout = timeout

    @staticmethod
    def _http_url(server_url: str) -> str:
        value = server_url.rstrip('/')
        if value.startswith('wss://'):
            return f'https://{value[6:]}'
        if value.startswith('ws://'):
            return f'http://{value[5:]}'
        return value

    def create(self, payload: dict[str, Any]) -> EnrollmentRequest:
        response = self._post(
            '/api/robot-registry/enrollments',
            payload,
            token=self.bootstrap_token,
        )
        return EnrollmentRequest(
            enrollment_id=str(response['enrollment_id']),
            pairing_code=str(response['pairing_code']),
            expires_at=str(response['expires_at']),
            poll_after_seconds=max(1, int(response.get('poll_after_seconds', 3))),
        )

    def claim(
        self,
        enrollment_id: str,
        pairing_code: str,
        hardware_fingerprint: str,
    ) -> AgentCredential:
        response = self._post(
            f'/api/robot-registry/enrollments/{enrollment_id}/claim',
            {
                'pairing_code': pairing_code,
                'hardware_fingerprint': hardware_fingerprint,
            },
        )
        return AgentCredential(
            robot_id=str(response['robot_id']),
            credential=str(response['credential']),
            credential_version=int(response['credential_version']),
            protocol_version=str(response.get('protocol_version', '1.0')),
        )

    def _post(
        self,
        path: str,
        payload: dict[str, Any],
        *,
        token: str | None = None,
    ) -> dict[str, Any]:
        headers = {'Content-Type': 'application/json'}
        if token:
            headers['Authorization'] = f'Bearer {token}'
        request = Request(
            f'{self.base_url}{path}',
            data=json.dumps(payload).encode('utf-8'),
            headers=headers,
            method='POST',
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                body = response.read().decode('utf-8')
        except HTTPError as error:
            try:
                detail = json.loads(error.read().decode('utf-8')).get('detail')
            except (json.JSONDecodeError, AttributeError):
                detail = None
            retry_after = None
            try:
                header = error.headers.get('Retry-After')
                if header is not None:
                    retry_after = max(1, int(float(header)))
            except (AttributeError, TypeError, ValueError):
                retry_after = None
            raise EnrollmentError(
                error.code,
                str(detail or error.reason),
                retry_after_seconds=retry_after,
            ) from error
        except URLError as error:
            raise EnrollmentError(0, str(error.reason)) from error
        try:
            result = json.loads(body)
        except json.JSONDecodeError as error:
            raise EnrollmentError(0, 'Enrollment server returned invalid JSON') from error
        if not isinstance(result, dict):
            raise EnrollmentError(0, 'Enrollment server returned an invalid response')
        return result


def machine_fingerprint(
    serial_number: str,
    machine_id_path: Path = Path('/etc/machine-id'),
) -> str:
    """Derive a stable non-secret fingerprint without exposing machine-id."""
    try:
        machine_id = machine_id_path.read_text(encoding='utf-8').strip()
    except OSError:
        machine_id = 'machine-id-unavailable'
    return hashlib.sha256(f'{machine_id}:{serial_number}'.encode('utf-8')).hexdigest()


def verification_fingerprint(hardware_fingerprint: str) -> str:
    """Return the digest displayed by FastAPI for physical verification."""
    return hashlib.sha256(hardware_fingerprint.encode('utf-8')).hexdigest()
