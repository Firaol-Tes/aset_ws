"""
Provider-agnostic LLM client wrapping Anthropic, OpenAI, and Gemini native
tool-calling behind one common interface, so planner_node.py doesn't need to
know which provider it's talking to.

anthropic/openai/google-genai live in the project venv, not the system Python
rclpy uses -- insert the venv's site-packages before importing them (same
pattern as mm_perception's lazy ultralytics import).
"""

import json
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

_VENV_SITE_PACKAGES = '/home/f/aset_ws/venv/lib/python3.12/site-packages'
if _VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, _VENV_SITE_PACKAGES)

from .tool_schemas import to_anthropic_tools, to_gemini_function_declarations, to_openai_tools  # noqa: E402


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: Dict[str, Any]


@dataclass
class ToolResultItem:
    id: str
    name: str
    content: str
    is_error: bool = False


@dataclass
class LLMTurnResult:
    tool_calls: List[ToolCall] = field(default_factory=list)
    assistant_text: str = ''
    input_tokens: int = 0
    output_tokens: int = 0
    stop_reason: str = ''


class BaseLLMClient:
    """self.raw_messages accumulates the provider-native conversation for logging."""

    def __init__(self, model: str, system_prompt: str):
        self.model = model
        self.system_prompt = system_prompt
        self.raw_messages: List[Dict[str, Any]] = []

    def start(self, user_text: str) -> LLMTurnResult:
        raise NotImplementedError

    def send_tool_results(self, results: List[ToolResultItem]) -> LLMTurnResult:
        raise NotImplementedError


class AnthropicClient(BaseLLMClient):
    def __init__(self, model: str, system_prompt: str, api_key: str):
        super().__init__(model, system_prompt)
        import anthropic
        self._client = anthropic.Anthropic(api_key=api_key)
        self._tools = to_anthropic_tools()
        self._messages: List[Dict[str, Any]] = []

    def _call(self) -> LLMTurnResult:
        response = self._client.messages.create(
            model=self.model,
            system=self.system_prompt,
            messages=self._messages,
            tools=self._tools,
            max_tokens=1024,
        )
        assistant_content = [block.model_dump() for block in response.content]
        self._messages.append({'role': 'assistant', 'content': assistant_content})
        self.raw_messages.append({'role': 'assistant', 'content': assistant_content})

        tool_calls = []
        text_parts = []
        for block in response.content:
            if block.type == 'tool_use':
                tool_calls.append(ToolCall(id=block.id, name=block.name, arguments=block.input))
            elif block.type == 'text':
                text_parts.append(block.text)

        return LLMTurnResult(
            tool_calls=tool_calls,
            assistant_text='\n'.join(text_parts),
            input_tokens=response.usage.input_tokens,
            output_tokens=response.usage.output_tokens,
            stop_reason=response.stop_reason or '',
        )

    def start(self, user_text: str) -> LLMTurnResult:
        msg = {'role': 'user', 'content': user_text}
        self._messages.append(msg)
        self.raw_messages.append(msg)
        return self._call()

    def send_tool_results(self, results: List[ToolResultItem]) -> LLMTurnResult:
        content = [
            {
                'type': 'tool_result',
                'tool_use_id': r.id,
                'content': r.content,
                'is_error': r.is_error,
            }
            for r in results
        ]
        msg = {'role': 'user', 'content': content}
        self._messages.append(msg)
        self.raw_messages.append(msg)
        return self._call()


class OpenAIClient(BaseLLMClient):
    def __init__(self, model: str, system_prompt: str, api_key: str):
        super().__init__(model, system_prompt)
        import openai
        self._client = openai.OpenAI(api_key=api_key)
        self._tools = to_openai_tools()
        self._messages: List[Dict[str, Any]] = []

    def _call(self) -> LLMTurnResult:
        full_messages = [{'role': 'system', 'content': self.system_prompt}] + self._messages
        response = self._client.chat.completions.create(
            model=self.model,
            messages=full_messages,
            tools=self._tools,
        )
        message = response.choices[0].message
        message_dict = {
            'role': 'assistant',
            'content': message.content,
            'tool_calls': (
                [tc.model_dump() for tc in message.tool_calls] if message.tool_calls else None
            ),
        }
        self._messages.append(message_dict)
        self.raw_messages.append(message_dict)

        tool_calls = []
        if message.tool_calls:
            for tc in message.tool_calls:
                try:
                    args = json.loads(tc.function.arguments)
                except json.JSONDecodeError:
                    args = {}
                tool_calls.append(ToolCall(id=tc.id, name=tc.function.name, arguments=args))

        return LLMTurnResult(
            tool_calls=tool_calls,
            assistant_text=message.content or '',
            input_tokens=response.usage.prompt_tokens,
            output_tokens=response.usage.completion_tokens,
            stop_reason=response.choices[0].finish_reason or '',
        )

    def start(self, user_text: str) -> LLMTurnResult:
        msg = {'role': 'user', 'content': user_text}
        self._messages.append(msg)
        self.raw_messages.append(msg)
        return self._call()

    def send_tool_results(self, results: List[ToolResultItem]) -> LLMTurnResult:
        for r in results:
            msg = {'role': 'tool', 'tool_call_id': r.id, 'content': r.content}
            self._messages.append(msg)
            self.raw_messages.append(msg)
        return self._call()


class GeminiClient(BaseLLMClient):
    def __init__(self, model: str, system_prompt: str, api_key: str):
        super().__init__(model, system_prompt)
        from google import genai
        from google.genai import types
        self._types = types
        self._client = genai.Client(api_key=api_key)
        self._config = types.GenerateContentConfig(
            tools=[types.Tool(function_declarations=to_gemini_function_declarations())],
            system_instruction=system_prompt,
        )
        self._contents: List[Any] = []

    @staticmethod
    def _to_loggable(content) -> Dict[str, Any]:
        try:
            return content.model_dump(exclude_none=True, mode='json')
        except Exception:
            return {'repr': str(content)}

    def _call(self) -> LLMTurnResult:
        response = self._client.models.generate_content(
            model=self.model,
            contents=self._contents,
            config=self._config,
        )
        model_content = response.candidates[0].content
        self._contents.append(model_content)
        self.raw_messages.append(self._to_loggable(model_content))

        tool_calls = []
        text_parts = []
        for part in model_content.parts:
            if getattr(part, 'function_call', None):
                fc = part.function_call
                tool_calls.append(ToolCall(id=fc.id or fc.name, name=fc.name, arguments=dict(fc.args or {})))
            elif getattr(part, 'text', None):
                text_parts.append(part.text)

        usage = getattr(response, 'usage_metadata', None)
        input_tokens = getattr(usage, 'prompt_token_count', 0) or 0
        output_tokens = getattr(usage, 'candidates_token_count', 0) or 0

        return LLMTurnResult(
            tool_calls=tool_calls,
            assistant_text='\n'.join(text_parts),
            input_tokens=input_tokens,
            output_tokens=output_tokens,
            stop_reason=str(getattr(response.candidates[0], 'finish_reason', '') or ''),
        )

    def start(self, user_text: str) -> LLMTurnResult:
        content = self._types.Content(role='user', parts=[self._types.Part(text=user_text)])
        self._contents.append(content)
        self.raw_messages.append(self._to_loggable(content))
        return self._call()

    def send_tool_results(self, results: List[ToolResultItem]) -> LLMTurnResult:
        parts = [
            self._types.Part.from_function_response(
                name=r.name,
                response={'error': r.content} if r.is_error else {'result': r.content},
            )
            for r in results
        ]
        content = self._types.Content(role='user', parts=parts)
        self._contents.append(content)
        self.raw_messages.append(self._to_loggable(content))
        return self._call()


def make_client(provider: str, model: str, system_prompt: str) -> BaseLLMClient:
    import os

    if provider == 'anthropic':
        api_key = os.environ.get('ANTHROPIC_API_KEY')
        if not api_key:
            raise RuntimeError('ANTHROPIC_API_KEY environment variable is not set')
        return AnthropicClient(model, system_prompt, api_key)
    elif provider == 'openai':
        api_key = os.environ.get('OPENAI_API_KEY')
        if not api_key:
            raise RuntimeError('OPENAI_API_KEY environment variable is not set')
        return OpenAIClient(model, system_prompt, api_key)
    elif provider == 'gemini':
        api_key = os.environ.get('GEMINI_API_KEY')
        if not api_key:
            raise RuntimeError('GEMINI_API_KEY environment variable is not set')
        return GeminiClient(model, system_prompt, api_key)
    else:
        raise ValueError(f'Unknown provider: {provider!r} (expected "anthropic", "openai", or "gemini")')
