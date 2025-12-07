# AIdd-book/code-examples/module-4/chapter3_llm_planning/llm_planner.py
import json
import os
from openai import OpenAI
from typing import Dict, Any

# It's assumed the API key is set in the environment variables
# For example: export OPENAI_API_KEY='your-key'
# You can also load it from a .env file using a library like python-dotenv

def get_llm_action_plan(natural_language_instruction: str, client: OpenAI = None) -> Dict[str, Any]:
    """
    Generates a sequence of ROS 2 actions from a natural language instruction
    using an LLM.

    This function constructs a detailed prompt for the LLM, asking it to act as
    a robotics assistant and decompose a high-level instruction into a JSON
    object representing a sequence of executable actions.

    If an API call fails or if the OPENAI_API_KEY is not set, it falls back
    to a simulated response for demonstration and testing purposes.

    Args:
        natural_language_instruction: The high-level command from the user.
        client: An optional OpenAI client instance. If not provided, a new one
                will be created using the environment API key.

    Returns:
        A dictionary representing the action plan, typically with an "actions" key.
    """
    if not client:
        # This will use the OPENAI_API_KEY environment variable by default
        client = OpenAI()

    # Enhanced prompt for better multi-step task decomposition
    prompt = f"""
    You are an expert roboticist's assistant. Your task is to convert a high-level
    natural language instruction into a structured JSON object representing a sequence
    of actions for a humanoid robot. Decompose complex instructions into a logical
    sequence of smaller, executable steps.

    The robot has the following capabilities, represented as action names:
    - "navigate_to_location(location: str)": Navigates to a named location (e.g., "kitchen", "living_room").
    - "identify_objects(objects: list[str])": Looks for and identifies specified objects (e.g., ["cup", "book"]).
    - "pick_up_object(object_type: str)": Picks up an object of a certain type (e.g., "apple").
    - "deposit_object(container: str)": Places the held object into a container (e.g., "trash_bin").
    - "place_object(location: str)": Places the held object on a surface at a location (e.g., "table").

    Instruction: "{natural_language_instruction}"

    Based on the instruction, generate a JSON object with a single key "actions".
    The value should be a list of action objects. Each object in the list must have
    a "name" (from the capabilities above) and a "parameters" dictionary.
    For example, for the instruction "Tidy the living room by throwing away the apple core",
    a good decomposition would be:
    1. Navigate to the living room.
    2. Find the apple core.
    3. Pick up the apple core.
    4. Navigate to the kitchen.
    5. Deposit the apple core in the trash bin.
    """

    try:
        if not os.getenv("OPENAI_API_KEY"):
            raise ValueError("OPENAI_API_KEY environment variable not set.")

        response = client.chat.completions.create(
            model="gpt-3.5-turbo", # Or another suitable model like gpt-4
            messages=[
                {"role": "system", "content": "You are a helpful assistant that translates natural language to robot action plans in JSON format. You are excellent at decomposing complex tasks."},
                {"role": "user", "content": prompt}
            ],
            response_format={"type": "json_object"}
        )
        plan_str = response.choices[0].message.content
        return json.loads(plan_str)

    except Exception as e:
        print(f"Error calling LLM API or OPENAI_API_KEY not set: {e}")
        # Fallback to a simulated response for testing without a live API key
        print("Using simulated fallback response.")
        if "clean the room" in natural_language_instruction.lower():
            return {
                "actions": [
                    {"name": "navigate_to_location", "parameters": {"location": "living_room"}},
                    {"name": "identify_objects", "parameters": {"objects": ["trash", "clutter"]}},
                    {"name": "pick_up_object", "parameters": {"object_type": "trash"}},
                    {"name": "deposit_object", "parameters": {"container": "trash_bin"}},
                    {"name": "pick_up_object", "parameters": {"object_type": "clutter"}},
                    {"name": "place_object", "parameters": {"location": "shelf"}},
                    {"name": "navigate_to_location", "parameters": {"location": "bedroom"}},
                ]
            }
        elif "apple" in natural_language_instruction.lower():
             return {
                "actions": [
                    {"name": "navigate_to_location", "parameters": {"location": "kitchen"}},
                    {"name": "identify_objects", "parameters": {"objects": ["apple"]}},
                    {"name": "pick_up_object", "parameters": {"object_type": "apple"}},
                    {"name": "navigate_to_location", "parameters": {"location": "operator"}},
                ]
            }
        return {"actions": []}


if __name__ == '__main__':
    # This is an example of how to use the function.
    # Make sure your OPENAI_API_KEY is set in your environment.
    instruction_1 = "Clean the room by picking up trash and putting away clutter."
    plan_1 = get_llm_action_plan(instruction_1)
    print("Generated Plan 1:")
    print(json.dumps(plan_1, indent=2))

    instruction_2 = "Bring me the apple from the kitchen table."
    plan_2 = get_llm_action_plan(instruction_2)
    print("\nGenerated Plan 2:")
    print(json.dumps(plan_2, indent=2))
