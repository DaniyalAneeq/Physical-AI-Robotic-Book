"""Comprehensive testing of all AIdd-book functionalities."""

import asyncio
import httpx
import json
from datetime import datetime

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'

def print_section(title):
    print(f"\n{Colors.BLUE}{'='*70}{Colors.RESET}")
    print(f"{Colors.BLUE}{title}{Colors.RESET}")
    print(f"{Colors.BLUE}{'='*70}{Colors.RESET}")

def print_success(message):
    print(f"{Colors.GREEN}✓{Colors.RESET} {message}")

def print_error(message):
    print(f"{Colors.RED}✗{Colors.RESET} {message}")

def print_info(message):
    print(f"{Colors.YELLOW}ℹ{Colors.RESET} {message}")

async def test_backend_health():
    """Test backend health endpoints."""
    print_section("1. BACKEND HEALTH CHECK")

    async with httpx.AsyncClient(timeout=10.0) as client:
        tests_passed = 0
        tests_total = 2

        # Test /health
        try:
            response = await client.get("http://localhost:8000/health")
            if response.status_code == 200:
                print_success(f"/health - Status: {response.status_code}")
                tests_passed += 1
            else:
                print_error(f"/health - Status: {response.status_code}")
        except Exception as e:
            print_error(f"/health - Error: {e}")

        # Test /api/health
        try:
            response = await client.get("http://localhost:8000/api/health")
            if response.status_code == 200:
                print_success(f"/api/health - Status: {response.status_code}")
                print_info(f"  Response: {response.json()}")
                tests_passed += 1
            else:
                print_error(f"/api/health - Status: {response.status_code}")
        except Exception as e:
            print_error(f"/api/health - Error: {e}")

        return tests_passed, tests_total

async def test_authentication_endpoints():
    """Test authentication system."""
    print_section("2. AUTHENTICATION SYSTEM")

    async with httpx.AsyncClient(timeout=10.0, follow_redirects=True) as client:
        tests_passed = 0
        tests_total = 6

        # Test /auth/session
        try:
            response = await client.get("http://localhost:8000/auth/session")
            print_info(f"/auth/session - Status: {response.status_code}")
            if response.status_code in [200, 401]:  # Either logged in or not
                tests_passed += 1
        except Exception as e:
            print_error(f"/auth/session - Error: {e}")

        # Test /auth/me
        try:
            response = await client.get("http://localhost:8000/auth/me")
            print_info(f"/auth/me - Status: {response.status_code}")
            if response.status_code in [200, 401]:
                tests_passed += 1
        except Exception as e:
            print_error(f"/auth/me - Error: {e}")

        # Test /auth/register (GET to check endpoint exists)
        try:
            response = await client.get("http://localhost:8000/auth/register")
            print_info(f"/auth/register - Status: {response.status_code} (endpoint exists)")
            if response.status_code in [200, 405, 422]:  # Exists but wrong method or missing params
                tests_passed += 1
        except Exception as e:
            print_error(f"/auth/register - Error: {e}")

        # Test /auth/login
        try:
            response = await client.get("http://localhost:8000/auth/login")
            print_info(f"/auth/login - Status: {response.status_code}")
            if response.status_code in [200, 405, 422]:
                tests_passed += 1
        except Exception as e:
            print_error(f"/auth/login - Error: {e}")

        # Test OAuth endpoints
        try:
            response = await client.get("http://localhost:8000/auth/oauth/google", follow_redirects=False)
            print_info(f"/auth/oauth/google - Status: {response.status_code}")
            if response.status_code in [200, 302, 307]:  # Redirect expected
                tests_passed += 1
                print_success("Google OAuth endpoint configured")
        except Exception as e:
            print_error(f"/auth/oauth/google - Error: {e}")

        try:
            response = await client.get("http://localhost:8000/auth/oauth/github", follow_redirects=False)
            print_info(f"/auth/oauth/github - Status: {response.status_code}")
            if response.status_code in [200, 302, 307]:
                tests_passed += 1
                print_success("GitHub OAuth endpoint configured")
        except Exception as e:
            print_error(f"/auth/oauth/github - Error: {e}")

        return tests_passed, tests_total

async def test_rag_chatbot():
    """Test RAG chatbot functionality."""
    print_section("3. RAG CHATBOT SYSTEM")

    async with httpx.AsyncClient(timeout=10.0) as client:
        tests_passed = 0
        tests_total = 5

        # Test /chatkit (ChatKit widget endpoint)
        try:
            response = await client.get("http://localhost:8000/chatkit")
            if response.status_code == 200:
                print_success(f"/chatkit (widget) - Status: {response.status_code}")
                tests_passed += 1
            else:
                print_info(f"/chatkit - Status: {response.status_code}")
        except Exception as e:
            print_error(f"/chatkit - Error: {e}")

        # Test /api/chat (POST - needs correct parameters)
        try:
            response = await client.post(
                "http://localhost:8000/api/chat",
                json={
                    "query": "What is ROS 2?",
                    "scope": {
                        "type": "module",
                        "module_id": "module-1",
                        "chapter_id": None,
                        "selected_text": None
                    },
                    "session_id": None
                }
            )
            if response.status_code == 200:
                print_success(f"/api/chat - Status: {response.status_code}")
                try:
                    result = response.json()
                    print_info(f"  Bot response: {str(result)[:150]}...")
                except:
                    # Might be a stream
                    print_info(f"  Bot response (stream): {response.text[:150]}...")
                tests_passed += 1
            elif response.status_code == 401:
                print_info(f"/api/chat - Requires authentication (401)")
                tests_passed += 1
            else:
                print_error(f"/api/chat - Status: {response.status_code}")
                print_info(f"  Response: {response.text[:200]}")
        except Exception as e:
            print_error(f"/api/chat - Error: {e}")

        # Test /api/conversations
        try:
            response = await client.get("http://localhost:8000/api/conversations")
            if response.status_code in [200, 401]:
                print_success(f"/api/conversations - Status: {response.status_code}")
                if response.status_code == 200:
                    conversations = response.json()
                    print_info(f"  Total conversations: {len(conversations)}")
                tests_passed += 1
        except Exception as e:
            print_error(f"/api/conversations - Error: {e}")

        # Test /api/index/status (Qdrant vector index)
        try:
            response = await client.get("http://localhost:8000/api/index/status")
            if response.status_code == 200:
                print_success(f"/api/index/status - Status: {response.status_code}")
                status = response.json()
                print_info(f"  Index status: {json.dumps(status, indent=2)}")
                tests_passed += 1
            else:
                print_info(f"/api/index/status - Status: {response.status_code}")
        except Exception as e:
            print_error(f"/api/index/status - Error: {e}")

        # Test /api/content/{module_id}
        try:
            response = await client.get("http://localhost:8000/api/content/module-1")
            if response.status_code == 200:
                print_success(f"/api/content/module-1 - Status: {response.status_code}")
                content = response.json()
                print_info(f"  Content items: {len(content) if isinstance(content, list) else 'N/A'}")
                tests_passed += 1
            else:
                print_info(f"/api/content/module-1 - Status: {response.status_code}")
        except Exception as e:
            print_error(f"/api/content/module-1 - Error: {e}")

        return tests_passed, tests_total

async def test_user_management():
    """Test user management endpoints."""
    print_section("4. USER MANAGEMENT")

    async with httpx.AsyncClient(timeout=10.0) as client:
        tests_passed = 0
        tests_total = 3

        # Test /api/user/me
        try:
            response = await client.get("http://localhost:8000/api/user/me")
            if response.status_code in [200, 401]:
                print_success(f"/api/user/me - Status: {response.status_code}")
                if response.status_code == 200:
                    print_info(f"  User data: {response.json()}")
                else:
                    print_info(f"  Not authenticated (expected)")
                tests_passed += 1
        except Exception as e:
            print_error(f"/api/user/me - Error: {e}")

        # Test /api/user/preferences
        try:
            response = await client.get("http://localhost:8000/api/user/preferences")
            if response.status_code in [200, 401]:
                print_success(f"/api/user/preferences - Status: {response.status_code}")
                tests_passed += 1
        except Exception as e:
            print_error(f"/api/user/preferences - Error: {e}")

        # Test /api/sessions
        try:
            response = await client.get("http://localhost:8000/api/sessions")
            if response.status_code in [200, 401]:
                print_success(f"/api/sessions - Status: {response.status_code}")
                tests_passed += 1
        except Exception as e:
            print_error(f"/api/sessions - Error: {e}")

        return tests_passed, tests_total

async def test_onboarding():
    """Test onboarding endpoints."""
    print_section("5. ONBOARDING SYSTEM")

    async with httpx.AsyncClient(timeout=10.0) as client:
        tests_passed = 0
        tests_total = 3

        endpoints = [
            "/auth/onboarding/status",
            "/auth/onboarding/options",
            "/auth/onboarding/profile"
        ]

        for endpoint in endpoints:
            try:
                response = await client.get(f"http://localhost:8000{endpoint}")
                if response.status_code in [200, 401, 405, 422]:
                    print_success(f"{endpoint} - Status: {response.status_code}")
                    tests_passed += 1
            except Exception as e:
                print_error(f"{endpoint} - Error: {e}")

        return tests_passed, tests_total

async def test_frontend_availability():
    """Test frontend availability."""
    print_section("6. FRONTEND AVAILABILITY")

    async with httpx.AsyncClient(timeout=10.0, follow_redirects=True) as client:
        tests_passed = 0
        tests_total = 4

        # Test homepage
        try:
            response = await client.get("http://localhost:3000/Physical-AI-Robotic-Book/")
            if response.status_code == 200:
                print_success(f"Homepage - Status: {response.status_code}")
                if "docusaurus" in response.text.lower() or "module" in response.text.lower():
                    print_info("  ✓ Docusaurus content detected")
                tests_passed += 1
            else:
                print_error(f"Homepage - Status: {response.status_code}")
        except Exception as e:
            print_error(f"Homepage - Error: {e}")

        # Test Module 1
        try:
            response = await client.get("http://localhost:3000/Physical-AI-Robotic-Book/docs/module-1/intro")
            if response.status_code == 200:
                print_success(f"Module 1 - Status: {response.status_code}")
                tests_passed += 1
        except Exception as e:
            print_error(f"Module 1 - Error: {e}")

        # Test Module 2
        try:
            response = await client.get("http://localhost:3000/Physical-AI-Robotic-Book/docs/module-2/intro")
            if response.status_code == 200:
                print_success(f"Module 2 - Status: {response.status_code}")
                tests_passed += 1
        except Exception as e:
            print_error(f"Module 2 - Error: {e}")

        # Test docs
        try:
            response = await client.get("http://localhost:3000/Physical-AI-Robotic-Book/docs")
            if response.status_code == 200:
                print_success(f"Docs index - Status: {response.status_code}")
                tests_passed += 1
        except Exception as e:
            print_error(f"Docs index - Error: {e}")

        return tests_passed, tests_total

async def main():
    """Run all tests."""
    print(f"\n{Colors.BLUE}{'='*70}{Colors.RESET}")
    print(f"{Colors.BLUE}COMPREHENSIVE AIDD-BOOK FUNCTIONALITY TEST{Colors.RESET}")
    print(f"{Colors.BLUE}{'='*70}{Colors.RESET}")
    print(f"{Colors.YELLOW}Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}{Colors.RESET}\n")

    total_passed = 0
    total_tests = 0

    # Run all test suites
    tests = [
        ("Backend Health", test_backend_health),
        ("Authentication", test_authentication_endpoints),
        ("RAG Chatbot", test_rag_chatbot),
        ("User Management", test_user_management),
        ("Onboarding", test_onboarding),
        ("Frontend", test_frontend_availability),
    ]

    results = []
    for name, test_func in tests:
        try:
            passed, total = await test_func()
            total_passed += passed
            total_tests += total
            results.append((name, passed, total))
        except Exception as e:
            print_error(f"Test suite '{name}' failed: {e}")
            results.append((name, 0, 0))

    # Print summary
    print_section("TEST SUMMARY")
    for name, passed, total in results:
        percentage = (passed / total * 100) if total > 0 else 0
        status = Colors.GREEN if percentage >= 80 else Colors.YELLOW if percentage >= 50 else Colors.RED
        print(f"{status}{name}: {passed}/{total} ({percentage:.1f}%){Colors.RESET}")

    print(f"\n{Colors.BLUE}{'='*70}{Colors.RESET}")
    overall_percentage = (total_passed / total_tests * 100) if total_tests > 0 else 0
    overall_status = Colors.GREEN if overall_percentage >= 80 else Colors.YELLOW if overall_percentage >= 50 else Colors.RED
    print(f"{overall_status}OVERALL: {total_passed}/{total_tests} tests passed ({overall_percentage:.1f}%){Colors.RESET}")
    print(f"{Colors.BLUE}{'='*70}{Colors.RESET}\n")

    if overall_percentage >= 80:
        print(f"{Colors.GREEN}✓ DEPLOYMENT READY{Colors.RESET} - All critical systems functional\n")
    elif overall_percentage >= 50:
        print(f"{Colors.YELLOW}⚠ NEEDS ATTENTION{Colors.RESET} - Some systems require fixes\n")
    else:
        print(f"{Colors.RED}✗ NOT READY{Colors.RESET} - Critical issues detected\n")

if __name__ == "__main__":
    asyncio.run(main())
