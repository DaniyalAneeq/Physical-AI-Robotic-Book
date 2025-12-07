import unittest
from unittest.mock import MagicMock, patch
from llm_planner import get_llm_action_plan

class TestLLMPlanner(unittest.TestCase):

    def test_get_llm_action_plan_simulated(self):
        """
        Tests the simulated response of the LLM planner when no API key is present.
        """
        instruction = "clean the room"
        expected_plan = {
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
        # We pass a client=None and the function should handle the fallback
        plan = get_llm_action_plan(instruction, client=None)
        self.assertEqual(plan, expected_plan)

    @patch('llm_planner.OpenAI')
    def test_get_llm_action_plan_with_mock_api(self, MockOpenAI):
        """
        Tests the LLM planner with a mocked OpenAI API call.
        """
        # Arrange
        mock_client = MagicMock()
        mock_response = MagicMock()
        mock_choice = MagicMock()
        mock_message = MagicMock()
        mock_message.content = '{"actions": [{"name": "navigate_to_location", "parameters": {"location": "test_location"}}]}'
        mock_choice.message = mock_message
        mock_response.choices = [mock_choice]
        mock_client.chat.completions.create.return_value = mock_response
        MockOpenAI.return_value = mock_client

        # Act
        instruction = "go to the test location"
        plan = get_llm_action_plan(instruction)

        # Assert
        mock_client.chat.completions.create.assert_called_once()
        self.assertIn("actions", plan)
        self.assertEqual(len(plan["actions"]), 1)
        self.assertEqual(plan["actions"][0]["name"], "navigate_to_location")
        self.assertEqual(plan["actions"][0]["parameters"]["location"], "test_location")

if __name__ == '__main__':
    unittest.main()
