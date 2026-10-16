"""Optional end-to-end check against running HTTP and reference navigator servers.

python TranDinhDuy-2A202602631/tests/river_browser_smoke.py [--browser /path/to/chrome] [--artifacts evidence/river]
Requires Playwright and an installed Chromium browser. See TranDinhDuy-2A202602631/viewer/river/INTEGRATION.md.
"""
import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8013/TranDinhDuy-2A202602631/viewer/river.html")
    parser.add_argument("--websocket-url", default="ws://127.0.0.1:8765")
    parser.add_argument("--browser", default=None)
    parser.add_argument("--artifacts", default=str(Path(__file__).resolve().parents[1] / "evidence" / "river"))
    args = parser.parse_args()
    artifacts = Path(args.artifacts)
    artifacts.mkdir(parents=True, exist_ok=True)
    errors = []
    with sync_playwright() as playwright:
        options = {"executable_path": args.browser} if args.browser else {}
        browser = playwright.chromium.launch(**options)
        page = browser.new_page(viewport={"width": 1440, "height": 960}, reduced_motion="reduce")
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.add_init_script("""
          window.__testSockets = [];
          window.WebSocket = class extends WebSocket {
            constructor(...args) { super(...args); window.__testSockets.push(this); }
          };
        """)
        page.goto(args.url)
        page.wait_for_function("window.RiverDemo && RiverDemo.getStatus().state.t === 0")
        observation = page.evaluate("RiverDemo.getObservation()")
        assert observation["metrics"]["distanceToEdge"] == 92.7
        assert observation["seq"] == 0

        page.locator("#timeline").fill("8.8")
        page.wait_for_timeout(150)
        state = page.evaluate("RiverDemo.getStatus().state")
        assert state["flightActive"] and state["y"] > 3
        page.screenshot(path=str(artifacts / "flying-desktop.png"))
        page.locator('[data-camera="follow"]').click()
        page.wait_for_timeout(500)
        page.screenshot(path=str(artifacts / "flying-follow.png"))
        page.locator('[data-camera="overview"]').click()

        page.locator("#timeline").fill(page.locator("#timeline").get_attribute("max"))
        assert page.evaluate("RiverDemo.getStatus().state.outcome") == "landed"
        with page.expect_download() as download:
            page.locator("#export").click()
        download.value.save_as(str(artifacts / "landed-run.json"))
        data = json.loads((artifacts / "landed-run.json").read_text(encoding="utf-8"))
        assert data["completed"] and data["outcome"] == "landed"
        assert data["frames"][-1]["state"]["terminal"]
        assert any(event["type"] == "rescue_activation" for event in data["events"])
        print("PASS mock flight, seek, cameras, completed export", flush=True)

        scenario_landings = []
        for name in ["direct", "wet", "spin"]:
            page.locator("#scenarioPreset").select_option(name)
            page.locator("#timeline").fill(page.locator("#timeline").get_attribute("max"))
            state = page.evaluate("RiverDemo.getStatus().state")
            assert state["outcome"] == "landed", (name, state)
            run = page.evaluate("RiverDemo.exportRun()")
            assert run["landingSummary"]["touchdown"] is not None
            if name == "spin":
                assert run["landingSummary"]["holdStartedAt"] is not None
            scenario_landings.append((round(state["x"],1),round(state["z"],1),round(state["yaw"],1)))
            page.screenshot(path=str(artifacts / ("scenario-"+name+".png")))
        assert len(set(scenario_landings)) == 3
        page.locator("#scenarioPreset").select_option("direct")
        page.locator("#timeline").fill("4.4")
        assert page.evaluate("RiverDemo.getStatus().state.skidding")
        page.screenshot(path=str(artifacts / "skidding.png"))
        page.locator("#timeline").fill("5.1")
        assert page.evaluate("RiverDemo.getStatus().state.mode") == "SPINNING"
        page.screenshot(path=str(artifacts / "ejected-spinning.png"))
        page.locator("#timeline").fill("5.3")
        assert page.evaluate("RiverDemo.getObservation().attitudeThrusters.yawPositive") > .1
        assert page.evaluate("RiverDemo.getObservation().thrusters.totalLift") == 0
        page.locator('[data-camera="follow"]').click()
        page.wait_for_timeout(200)
        page.screenshot(path=str(artifacts / "body-jets.png"))
        page.locator("#timeline").fill("5.8")
        assert page.evaluate("RiverDemo.getObservation().thrusters.totalLift") > .1
        page.screenshot(path=str(artifacts / "underbody-lift.png"))
        print("PASS scenario presets land differently, skid and body-jet recovery are visible", flush=True)

        page.locator("#flightOff").click()
        page.locator("#timeline").fill(page.locator("#timeline").get_attribute("max"))
        assert page.evaluate("RiverDemo.getStatus().state.outcome") == "splash"
        page.screenshot(path=str(artifacts / "splash-desktop.png"))
        print("PASS road-only falls into river", flush=True)

        page.locator("#flightOn").click()
        page.locator("#riverWidth").fill("48")
        page.locator("#riverWidth").dispatch_event("change")
        page.locator("#timeline").fill(page.locator("#timeline").get_attribute("max"))
        assert page.evaluate("RiverDemo.getObservation().map.farBankX") == 48
        assert page.evaluate("RiverDemo.getStatus().state.outcome") == "landed"
        page.locator("#riverWidth").fill("24")
        page.locator("#riverWidth").dispatch_event("change")
        page.locator("#timeline").fill("8.8")
        print("PASS scenario reconfiguration rebuilds geometry and route", flush=True)

        page.set_viewport_size({"width": 390, "height": 844})
        page.wait_for_timeout(200)
        assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
        page.screenshot(path=str(artifacts / "flying-mobile.png"), full_page=True)
        page.set_viewport_size({"width": 1366, "height": 768})
        page.wait_for_timeout(200)
        assert page.evaluate("document.querySelector('.scenario-panel').getBoundingClientRect().bottom < document.querySelector('.transport').getBoundingClientRect().top")
        page.screenshot(path=str(artifacts / "flying-laptop.png"))
        print("PASS mobile and laptop layouts", flush=True)

        model = page.evaluate("RiverDemo.inspectVehicle()")
        assert len(model["thrusters"]) == 4
        assert all(nozzle["y"] < 0.5 for nozzle in model["thrusters"])
        assert {nozzle["name"] for nozzle in model["thrusters"]} == {
            "FrontLeftThruster", "FrontRightThruster", "RearLeftThruster", "RearRightThruster"}
        assert len(model["attitudeJets"]) == 8
        page.locator("#debugPanel summary").click()
        page.locator("#showRays").check()
        page.locator("#autoRescue").uncheck()
        page.locator("#forceFall").click()
        page.wait_for_function("RiverDemo.getStatus().state.mode === 'SPINNING'")
        page.locator("#viewport canvas").click(position={"x": 500, "y": 300})
        page.keyboard.press("Space")
        page.wait_for_function("RiverDemo.getStatus().state.flightActive")
        page.wait_for_function("Math.abs(RiverDemo.getStatus().state.pitch)<0.04 && Math.abs(RiverDemo.getStatus().state.roll)<0.04")
        page.keyboard.press("h")
        assert page.evaluate("RiverDemo.getStatus().state.hoverMode")
        page.keyboard.press("l")
        assert page.evaluate("RiverDemo.getStatus().state.forceLanding")
        page.screenshot(path=str(artifacts / "manual-rescue.png"))
        page.keyboard.press("r")
        page.locator("#autoRescue").check()
        page.locator("#useMock").click()
        page.locator("#debugPanel summary").click()
        page.locator("#play").click()  # Pause before checking WASD steering.
        page.locator("#viewport canvas").click(position={"x": 500, "y": 300})
        page.keyboard.down("w")
        page.keyboard.down("d")
        page.wait_for_function("RiverDemo.getStatus().state.z > 0.2")
        page.keyboard.up("w")
        page.keyboard.up("d")
        print("PASS underbody emitters, sensor debug, manual rescue, hover/landing and WASD", flush=True)

        # A custom controller that brakes immediately must override the mock's route.
        page.evaluate("RiverDemo.setController({name:'Early brake test',reset(){},update(){return {brake:1};}})")
        page.locator("#playbackSpeed").click()  # 2x
        page.wait_for_function("RiverDemo.getStatus().state.terminal", timeout=15000)
        assert page.evaluate("RiverDemo.getStatus().state.outcome") == "stopped"
        assert page.locator("#timeline").is_disabled()
        print("PASS custom JavaScript controller controls physics", flush=True)

        page.locator("#socketUrl").fill(args.websocket_url)
        page.locator("#connect").click()
        page.wait_for_function("RiverDemo.getStatus().state.terminal", timeout=30000)
        assert page.evaluate("RiverDemo.getStatus().state.outcome") == "landed", page.evaluate("RiverDemo.getStatus()")
        assert page.evaluate("RiverDemo.getStatus().mode") == "socket"
        assert page.locator("#timeline").is_disabled()
        page.screenshot(path=str(artifacts / "websocket-landed.png"))
        print("PASS real WebSocket round-trip reaches landing target", flush=True)

        page.locator("#restart").click()
        page.wait_for_function("RiverDemo.getStatus().state.t > 1")
        page.evaluate("window.__testSockets.at(-1).close()")
        page.wait_for_timeout(200)
        stopped_time = page.evaluate("RiverDemo.getStatus().state.t")
        page.wait_for_timeout(600)
        assert page.evaluate("RiverDemo.getStatus().state.t") == stopped_time
        assert page.evaluate("RiverDemo.getStatus().waiting")
        page.locator("#useMock").click()
        page.wait_for_timeout(100)
        assert page.evaluate("RiverDemo.getStatus().mode") == "mock"
        assert "Mock" in page.locator("#connectionStatus").inner_text()
        print("PASS disconnect freezes simulation, explicit mock switch recovers", flush=True)
        assert not errors, errors
        print("PASS no browser JavaScript errors", flush=True)
        browser.close()


if __name__ == "__main__":
    main()
