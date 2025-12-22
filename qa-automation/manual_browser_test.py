"""Manual browser testing script to verify all functionalities."""

import asyncio
import sys
from playwright.async_api import async_playwright
import httpx

async def test_backend_apis():
    """Test all backend APIs."""
    print("\n" + "="*60)
    print("TESTING BACKEND APIs")
    print("="*60)

    base_url = "https://e-book-physical-ai-humanoid-robotics.onrender.com"

    async with httpx.AsyncClient() as client:
        # Test health endpoint
        print("\n1. Testing /health endpoint...")
        try:
            response = await client.get(f"{base_url}/health")
            print(f"   ✓ Status: {response.status_code}")
            print(f"   Response: {response.text[:100]}")
        except Exception as e:
            print(f"   ✗ Error: {e}")

        # Test docs endpoint
        print("\n2. Testing /docs endpoint...")
        try:
            response = await client.get(f"{base_url}/docs")
            print(f"   ✓ Status: {response.status_code}")
        except Exception as e:
            print(f"   ✗ Error: {e}")

        # Test auth endpoints
        print("\n3. Testing auth endpoints...")
        auth_endpoints = [
            "/auth/sign-in/email",
            "/auth/sign-up/email",
            "/auth/session",
        ]
        for endpoint in auth_endpoints:
            try:
                response = await client.get(f"{base_url}{endpoint}")
                print(f"   {endpoint}: {response.status_code}")
            except Exception as e:
                print(f"   {endpoint}: Error - {e}")

        # Test chatkit endpoint
        print("\n4. Testing chatkit endpoints...")
        try:
            response = await client.get(f"{base_url}/chatkit/api/chat")
            print(f"   /chatkit/api/chat: {response.status_code}")
        except Exception as e:
            print(f"   /chatkit/api/chat: Error - {e}")

async def test_frontend_browser():
    """Test frontend with real browser."""
    print("\n" + "="*60)
    print("TESTING FRONTEND WITH BROWSER")
    print("="*60)

    async with async_playwright() as p:
        # Launch browser in headed mode (visible)
        print("\n1. Launching browser...")
        try:
            browser = await p.chromium.launch(headless=False, slow_mo=1000)
            context = await browser.new_context()
            page = await context.new_page()

            # Test homepage
            print("\n2. Testing homepage...")
            await page.goto("http://localhost:3000/Physical-AI-Robotic-Book/")
            await page.wait_for_load_state("networkidle")
            title = await page.title()
            print(f"   ✓ Page title: {title}")

            # Check for main elements
            print("\n3. Checking page elements...")
            h1_exists = await page.locator("h1").count() > 0
            print(f"   H1 heading: {'✓ Found' if h1_exists else '✗ Missing'}")

            nav_exists = await page.locator("nav").count() > 0
            print(f"   Navigation: {'✓ Found' if nav_exists else '✗ Missing'}")

            # Take screenshot
            await page.screenshot(path="homepage-manual-test.png", full_page=True)
            print("   ✓ Screenshot saved: homepage-manual-test.png")

            # Test Module 1 page
            print("\n4. Testing Module 1 page...")
            try:
                await page.goto("http://localhost:3000/Physical-AI-Robotic-Book/docs/module-1/intro")
                await page.wait_for_load_state("networkidle")
                article_exists = await page.locator("article").count() > 0
                print(f"   Article content: {'✓ Found' if article_exists else '✗ Missing'}")
                await page.screenshot(path="module1-manual-test.png", full_page=True)
                print("   ✓ Screenshot saved: module1-manual-test.png")
            except Exception as e:
                print(f"   ✗ Error: {e}")

            # Test chatbot if visible
            print("\n5. Testing RAG Chatbot...")
            try:
                # Look for chatbot elements
                chatbot_button = page.locator("button:has-text('Chat'), [aria-label*='chat']")
                count = await chatbot_button.count()
                if count > 0:
                    print(f"   ✓ Found {count} chat-related button(s)")
                    await chatbot_button.first.click()
                    await page.wait_for_timeout(2000)
                    await page.screenshot(path="chatbot-open.png", full_page=True)
                    print("   ✓ Chatbot interface screenshot saved")
                else:
                    print("   ℹ No visible chatbot button found")
            except Exception as e:
                print(f"   ℹ Chatbot test: {e}")

            # Test authentication
            print("\n6. Testing Authentication UI...")
            try:
                # Look for auth elements
                auth_elements = page.locator("button:has-text('Sign'), button:has-text('Login'), a:has-text('Login')")
                count = await auth_elements.count()
                if count > 0:
                    print(f"   ✓ Found {count} auth-related element(s)")
                else:
                    print("   ℹ No visible auth buttons found (might be in menu)")
            except Exception as e:
                print(f"   ℹ Auth test: {e}")

            # Check console errors
            print("\n7. Checking for console errors...")
            page.on("console", lambda msg: print(f"   Console [{msg.type}]: {msg.text}") if msg.type in ["error", "warning"] else None)

            # Wait a bit for any async errors
            await page.wait_for_timeout(3000)

            print("\n8. Keeping browser open for 10 seconds for manual inspection...")
            await page.wait_for_timeout(10000)

            await browser.close()
            print("\n✓ Browser test completed")

        except Exception as e:
            print(f"\n✗ Browser test failed: {e}")
            print(f"Error type: {type(e).__name__}")
            import traceback
            traceback.print_exc()

async def main():
    """Run all tests."""
    print("\n" + "="*60)
    print("COMPREHENSIVE MANUAL TESTING")
    print("="*60)

    # Test backend first
    await test_backend_apis()

    # Test frontend with browser
    await test_frontend_browser()

    print("\n" + "="*60)
    print("ALL TESTS COMPLETED")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(main())
