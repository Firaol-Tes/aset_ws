import sys
sys.path.insert(0, '/home/f/aset_ws/install/mm_llm_planner/lib/python3.12/site-packages')
sys.path.insert(0, '/home/f/aset_ws/install/mm_interfaces/lib/python3.12/site-packages')
from mm_llm_planner.llm_clients import GeminiClient
g = GeminiClient('gemini-3.5-flash', 'test prompt', 'dummy-key')
print('GeminiClient OK, function declarations:', len(g._config.tools[0].function_declarations))
