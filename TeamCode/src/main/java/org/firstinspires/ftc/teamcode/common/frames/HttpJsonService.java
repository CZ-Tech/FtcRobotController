package org.firstinspires.ftc.teamcode.common.util;

import static java.nio.charset.StandardCharsets.UTF_8;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeManager;
import org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.TaskLoopFrame;
import org.firstinspires.ftc.teamcode.opmode.auto.JsonPathOpMode;
import org.json.JSONArray;
import org.json.JSONObject;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.drive.MixedOdo;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * A simple HTTP service that starts automatically when the RC App launches (via @OpModeRegistrar),
 * allowing external clients to send and retrieve JSON data over HTTP on port 8888.
 * Supports named paths so multiple configs/routes can be stored independently.
 *
 * <h2>API Endpoints</h2>
 * <table border="1">
 *   <tr><th>Method</th><th>Path</th><th>Request Body</th><th>Response</th><th>Description</th></tr>
 *   <tr><td>OPTIONS</td><td>Any</td><td>None</td><td>204 No Content</td><td>CORS preflight</td></tr>
 *   <tr><td>POST</td><td>/</td><td>JSON string</td><td>{"status":"success"}</td><td>Receive JSON into current memory</td></tr>
 *   <tr><td>GET</td><td>/</td><td>None</td><td>Current JSON</td><td>Returns the JSON currently in memory</td></tr>
 *   <tr><td>POST</td><td>/save/{name}</td><td>None</td><td>{"status":"saved","path":"{name}"}</td><td>Persist current JSON under a named path</td></tr>
 *   <tr><td>GET</td><td>/{name}</td><td>None</td><td>Saved JSON (200) or not_found (404)</td><td>Read saved JSON without changing current</td></tr>
 *   <tr><td>POST</td><td>/load/{name}</td><td>None</td><td>{"status":"loaded","path":"{name}","data":...}</td><td>Load saved JSON into current memory</td></tr>
 *   <tr><td>POST</td><td>/clear/{name}</td><td>None</td><td>{"status":"cleared","path":"{name}"}</td><td>Delete a named path</td></tr>
 *   <tr><td>GET</td><td>/list</td><td>None</td><td>{"status":"ok","paths":[...]}</td><td>List all saved path names</td></tr>
 *   <tr><td>GET</td><td>/commands</td><td>None</td><td>{"status":"ok","commands":[...]}</td><td>List all registered {@link AutoTask} commands with signatures</td></tr>
 *   <tr><td>GET</td><td>/position</td><td>None</td><td>{"status":"ok","x":...,"y":...,"heading":...,"unit":"inches","headingUnit":"degrees"}</td><td>Returns the robot's current field center coordinates (requires active Robot)</td></tr>
 *   <tr><td>GET</td><td>/status</td><td>None</td><td>{"status":"ok","opModeActive":...,"executionReady":...,"isExecuting":...,"activeOpModeName":...,"commandCount":...,"commandsReady":...}</td><td>Returns the current OpMode and execution state for external tooling (AzConductor, etc.)</td></tr>
 *   <tr><td>POST</td><td>/commands/run/{name}</td><td>JSON array of args</td><td>{"status":"ok"}</td><td>Invoke a registered command by name</td></tr>
 *   <tr><td>POST</td><td>/run/saved/{pathName}</td><td>None</td><td>{"status":"ok","path":"{pathName}"} (200) / error (404/409/503)</td><td>Queue a saved path for execution (requires HttpAuto OpMode active)</td></tr>
 *   <tr><td>POST</td><td>/run/temp</td><td>JSON path</td><td>{"status":"ok"} (200) / error (400/409/503)</td><td>Queue a temporary path from request body for execution (requires HttpAuto OpMode active)</td></tr>
 * </table>
 *
 * <h2>CORS</h2>
 * All responses include CORS headers (Access-Control-Allow-Origin: *, etc.) so browser-based
 * clients can call the API from any origin without being blocked.
 *
 * <h2>Typical Workflow</h2>
 * <pre>
 *   curl -X POST http://&lt;robot-ip&gt;:8888/ -d '{"route":"A","waypoints":[...]}'
 *   curl -X POST http://&lt;robot-ip&gt;:8888/save/routeA
 *   curl -X POST http://&lt;robot-ip&gt;:8888/ -d '{"route":"B","waypoints":[...]}'
 *   curl -X POST http://&lt;robot-ip&gt;:8888/save/routeB
 *   curl http://&lt;robot-ip&gt;:8888/list
 *   curl -X POST http://&lt;robot-ip&gt;:8888/load/routeA
 *   curl -X POST http://&lt;robot-ip&gt;:8888/clear/routeB
 * </pre>
 *
 * <h2>Persistence</h2>
 * Each path is stored as a separate SharedPreferences key ({@code http_saved_json_{pathName}}),
 * surviving app updates. Only a full uninstall or "Clear Data" removes them.
 *
 * <h2>AutoCommand System</h2>
 * Classes and methods annotated with {@code @AutoTask} are automatically registered as
 * HTTP-invokable commands when {@link #scanObjectTree(Object)} is called (triggered internally
 * by {@code TrajectoryLoader.execute()}). Invoked commands run on independent threads via
 * {@code TaskLoopFrame.runOnce()}.
 *
 * <h2>Thread Safety</h2>
 * {@code currentJson}, {@code executionReady}, {@code pendingExecution}, and {@code pendingPathJson}
 * are volatile; the path cache and command registry use ConcurrentHashMap.
 * Path execution commands are written by the HTTP daemon thread and consumed by the OpMode thread.
 * Server runs on a daemon min-priority thread.
 */
public class HttpJsonService {
    private static final String TAG = "HttpJsonService";
    private static final int PORT = 8888;
    private static final String PREF_KEY_PREFIX = "http_saved_json_";

    private static final String JSON_OPMODE_GROUP = "JSON Routes";
    private static boolean isStarted = false;
    private static Context appContext;
    private static RegisteredOpModes registeredOpModes;

    /** Current in-memory JSON. volatile for cross-thread visibility. */
    private static volatile String currentJson = "{}";
    /** In-memory cache: pathName → saved JSON. ConcurrentHashMap for thread safety. */
    private static final ConcurrentHashMap<String, String> pathCache = new ConcurrentHashMap<>();

    /** Command registry: commandName → CommandEntry. Populated by scanObjectTree(). */
    private static final ConcurrentHashMap<String, CommandEntry> commandRegistry = new ConcurrentHashMap<>();
    /** Tracks which root objects have already been scanned (by identity hash). */
    private static final Set<Integer> scannedRoots = new HashSet<>();

    /** Active Robot instance for serving live telemetry (e.g. GET /position). volatile for cross-thread visibility. */
    private static volatile Robot activeRobot;

    /** Flag set by HttpAuto OpMode to indicate it is ready to execute paths via HTTP. volatile for cross-thread visibility. */
    private static volatile boolean executionReady = false;
    /** Flag set by HTTP endpoint when a path is queued for execution. Cleared by the OpMode thread after consuming. volatile for cross-thread visibility. */
    private static volatile boolean pendingExecution = false;
    /** JSON content for the pending path execution. Set by HTTP endpoint, consumed by OpMode thread. volatile for cross-thread visibility. */
    private static volatile String pendingPathJson = null;

    /** Name of the currently active OpMode (null if none). Set by OpModes themselves so external clients can identify which OpMode is running. volatile for cross-thread visibility. */
    private static volatile String activeOpModeName = null;

    /** Holds a registered command's invocation metadata. */
    static class CommandEntry {
        final String name;
        final String matchKey;
        volatile Object instance;
        final Method method;
        final Class<?>[] paramTypes;
        final String[] paramNames;
        volatile boolean ready;

        CommandEntry(String name, String matchKey, Object instance, Method method,
                  Class<?>[] paramTypes, String[] paramNames, boolean ready) {
            this.name = name;
            this.matchKey = matchKey;
            this.instance = instance;
            this.method = method;
            this.paramTypes = paramTypes;
            this.paramNames = paramNames;
            this.ready = ready;
        }
    }

    @OpModeRegistrar
    public static void initService(Context context, OpModeManager manager) {
        if (!isStarted) {
            appContext = context.getApplicationContext();
            loadAllPathsFromDisk();

            // Store reference to RegisteredOpModes for dynamic updates
            registeredOpModes = RegisteredOpModes.getInstance();

            // Register a registrar that provides JsonPathOpMode instances for each saved path.
            // Called once at startup, and again whenever registerInstanceOpModes() is invoked.
            registeredOpModes.addInstanceOpModeRegistrar(new InstanceOpModeRegistrar() {
                @Override
                public void register(InstanceOpModeManager manager) {
                    for (String pathName : pathCache.keySet()) {
                        String opModeName = "Json:" + pathName;
                        OpModeMeta meta = new OpModeMeta.Builder()
                                .setName(opModeName)
                                .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                                .setGroup(JSON_OPMODE_GROUP)
                                .build();
                        manager.register(meta, new JsonPathOpMode(pathName));
                        Log.d(TAG, "Registered dynamic OpMode: " + opModeName);
                    }
                }
            });

            // Trigger initial registration: the SDK has already passed the point
            // where InstanceOpModeRegistrars are called, so we must call it ourselves.
            registeredOpModes.registerInstanceOpModes();

            scanClassTree(Robot.class);

            Thread serverThread = new Thread(HttpJsonService::runServer);
            serverThread.setPriority(Thread.MIN_PRIORITY);
            serverThread.setDaemon(true);
            serverThread.start();
            isStarted = true;
            Log.i(TAG, "--- HTTP JSON Service started, listening on port: " + PORT + " ---");
        }
    }

    private static void runServer() {
        try (ServerSocket serverSocket = new ServerSocket(PORT)) {
            while (!Thread.currentThread().isInterrupted()) {
                try (Socket socket = serverSocket.accept()) {
                    socket.setSoTimeout(5000);
                    try (InputStream in = socket.getInputStream();
                         OutputStream output = socket.getOutputStream()) {

                        try {
                            String requestLine = readLineFromStream(in);
                            if (requestLine == null || requestLine.isEmpty()) continue;

                            boolean isPost    = requestLine.startsWith("POST");
                            boolean isGet     = requestLine.startsWith("GET");
                            boolean isOptions = requestLine.startsWith("OPTIONS");
                            String fullPath = parseRequestPath(requestLine);

                            int contentLength = 0;
                            String headerLine;
                            while (!(headerLine = readLineFromStream(in)).isEmpty()) {
                                if (headerLine.toLowerCase().startsWith("content-length:")) {
                                    try {
                                        contentLength = Integer.parseInt(headerLine.substring(15).trim());
                                    } catch (Exception ignored) {}
                                }
                            }

                            // Read body as BYTES (not chars) so Content-Length works correctly for UTF-8
                            String body = "";
                            if (contentLength > 0) {
                                byte[] bodyBytes = readBodyBytes(in, contentLength);
                                body = new String(bodyBytes, UTF_8);
                            }

                            // --- OPTIONS preflight — CORS ---
                            if (isOptions) {
                                sendResponse(output, 204, null, null);
                                continue;
                            }

                            // --- GET / — return current JSON ---
                            if (isGet && "/".equals(fullPath)) {
                                sendResponse(output, 200, "application/json", currentJson);
                                continue;
                            }

                            // --- POST / (with body) — receive JSON into current memory ---
                            if (isPost && "/".equals(fullPath) && contentLength > 0) {
                                currentJson = body;
                                Log.d(TAG, "Received JSON: " + currentJson);
                                sendResponse(output, 200, "application/json", "{\"status\":\"success\"}");
                                continue;
                            }

                            // --- Parse /action/pathName ---
                            String[] parts = fullPath.split("/", 3);
                            String action   = parts.length >= 2 ? parts[1] : "";
                            String pathName = parts.length >= 3 ? parts[2] : "";

                            // --- POST /save/{pathName} ---
                            if (isPost && "save".equals(action) && !pathName.isEmpty()) {
                                pathCache.put(pathName, currentJson);
                                saveToDisk(pathName, currentJson);
                                refreshDynamicOpModes();
                                Log.i(TAG, "Saved path '" + pathName + "': " + currentJson);
                                sendResponse(output, 200, "application/json",
                                        "{\"status\":\"saved\",\"path\":\"" + pathName + "\"}");
                                continue;
                            }

                            // --- GET /{pathName} — read saved JSON for a path
                            // pathName may be in action (e.g. GET /myPath) or pathName (e.g. GET //myPath)
                            if (isGet && !action.isEmpty() && pathName.isEmpty()
                                    && !"list".equals(action) && !"commands".equals(action)
                                    && !"position".equals(action) && !"status".equals(action)
                                    && !"run".equals(action)) {
                                String saved = pathCache.get(action);
                                if (saved != null) {
                                    sendResponse(output, 200, "application/json", saved);
                                } else {
                                    sendResponse(output, 404, "application/json",
                                            "{\"status\":\"not_found\",\"path\":\"" + action + "\"}");
                                }
                                continue;
                            }

                            // --- POST /load/{pathName} ---
                            if (isPost && "load".equals(action) && !pathName.isEmpty()) {
                                String saved = pathCache.get(pathName);
                                if (saved != null) {
                                    currentJson = saved;
                                    Log.i(TAG, "Loaded path '" + pathName + "': " + currentJson);
                                    sendResponse(output, 200, "application/json",
                                            "{\"status\":\"loaded\",\"path\":\"" + pathName + "\",\"data\":" + currentJson + "}");
                                } else {
                                    sendResponse(output, 404, "application/json",
                                            "{\"status\":\"not_found\",\"path\":\"" + pathName + "\"}");
                                }
                                continue;
                            }

                            // --- POST /clear/{pathName} ---
                            if (isPost && "clear".equals(action) && !pathName.isEmpty()) {
                                pathCache.remove(pathName);
                                deleteFromDisk(pathName);
                                refreshDynamicOpModes();
                                Log.i(TAG, "Cleared path '" + pathName + "'");
                                sendResponse(output, 200, "application/json",
                                        "{\"status\":\"cleared\",\"path\":\"" + pathName + "\"}");
                                continue;
                            }

                            // --- GET /list ---
                            if (isGet && "list".equals(action)) {
                                List<String> list = new ArrayList<>(pathCache.keySet());
                                StringBuilder sb = new StringBuilder("{\"status\":\"ok\",\"paths\":[");
                                for (int i = 0; i < list.size(); i++) {
                                    if (i > 0) sb.append(",");
                                    sb.append("\"").append(escapeJson(list.get(i))).append("\"");
                                }
                                sb.append("]}");
                                sendResponse(output, 200, "application/json", sb.toString());
                                continue;
                            }

                            // --- GET /commands ---
                            if (isGet && "commands".equals(action) && pathName.isEmpty()) {
                                StringBuilder sb = new StringBuilder("{\"status\":\"ok\",\"commands\":[");
                                boolean first = true;
                                for (CommandEntry entry : commandRegistry.values()) {
                                    if (!first) sb.append(",");
                                    first = false;
                                    sb.append("{\"name\":\"").append(escapeJson(entry.name))
                                      .append("\",\"params\":[");
                                    for (int i = 0; i < entry.paramTypes.length; i++) {
                                        if (i > 0) sb.append(",");
                                        sb.append("\"").append(typeName(entry.paramTypes[i])).append("\"");
                                    }
                                    sb.append("],\"paramNames\":[");
                                    for (int i = 0; i < entry.paramNames.length; i++) {
                                        if (i > 0) sb.append(",");
                                        sb.append("\"").append(escapeJson(entry.paramNames[i])).append("\"");
                                    }
                                    sb.append("],\"ready\":").append(entry.ready).append("}");
                                }
                                sb.append("]}");
                                sendResponse(output, 200, "application/json", sb.toString());
                                continue;
                            }

                            // --- GET /position ---
                            if (isGet && "position".equals(action)) {
                                Robot robot = activeRobot;
                                if (robot == null) {
                                    sendResponse(output, 503, "application/json",
                                            "{\"status\":\"error\",\"message\":\"Robot not initialized\"}");
                                    continue;
                                }
                                MixedOdo odo = robot.odo;
                                if (odo == null) {
                                    sendResponse(output, 503, "application/json",
                                            "{\"status\":\"error\",\"message\":\"Odometry not initialized\"}");
                                    continue;
                                }
                                Pose2D pos = odo.getPosition();
                                if (pos == null) {
                                    sendResponse(output, 503, "application/json",
                                            "{\"status\":\"error\",\"message\":\"Position unavailable\"}");
                                    continue;
                                }
                                double x = pos.getX(DistanceUnit.INCH);
                                double y = pos.getY(DistanceUnit.INCH);
                                double heading = pos.getHeading(AngleUnit.DEGREES);
                                String json = "{\"status\":\"ok\"," +
                                        "\"x\":" + String.format(Locale.US, "%.2f", x) + "," +
                                        "\"y\":" + String.format(Locale.US, "%.2f", y) + "," +
                                        "\"heading\":" + String.format(Locale.US, "%.2f", heading) + "," +
                                        "\"unit\":\"inches\"," +
                                        "\"headingUnit\":\"degrees\"}";
                                sendResponse(output, 200, "application/json", json);
                                continue;
                            }

                            // --- GET /status ---
                            if (isGet && "status".equals(action) && pathName.isEmpty()) {
                                boolean opModeActive = activeRobot != null;
                                boolean isExecuting = pendingExecution;
                                String opModeName = activeOpModeName;
                                int totalCommands = commandRegistry.size();
                                int commandsReady = countReady();

                                String json = "{" +
                                        "\"status\":\"ok\"," +
                                        "\"opModeActive\":" + opModeActive + "," +
                                        "\"executionReady\":" + executionReady + "," +
                                        "\"isExecuting\":" + isExecuting + "," +
                                        "\"activeOpModeName\":" + (opModeName != null ? "\"" + escapeJson(opModeName) + "\"" : "null") + "," +
                                        "\"commandCount\":" + totalCommands + "," +
                                        "\"commandsReady\":" + commandsReady +
                                        "}";
                                sendResponse(output, 200, "application/json", json);
                                continue;
                            }

                            // --- POST /commands/run/{commandName} ---
                            if (isPost && "commands".equals(action) && pathName.startsWith("run/")) {
                                String commandName = pathName.substring(4);
                                CommandEntry entry = commandRegistry.get(commandName);
                                if (entry == null) {
                                    sendResponse(output, 404, "application/json",
                                            "{\"status\":\"error\",\"message\":\"Command not found: " + escapeJson(commandName) + "\"}");
                                    continue;
                                }
                                if (!entry.ready || entry.instance == null) {
                                    sendResponse(output, 503, "application/json",
                                            "{\"status\":\"error\",\"message\":\"Command not ready: " + escapeJson(commandName) + " (no active OpMode)\"}");
                                    continue;
                                }
                                try {
                                    String commandBody = (contentLength > 0) ? body : "[]";
                                    JSONArray jsonArgs = new JSONArray(commandBody);
                                    Object[] args = convertArgs(jsonArgs, entry.paramTypes);
                                    TaskLoopFrame.runOnce(() -> {
                                        try {
                                            entry.method.invoke(entry.instance, args);
                                        } catch (Exception e) {
                                            Log.e(TAG, "Command invocation failed: " + entry.name, e);
                                        }
                                    });
                                    sendResponse(output, 200, "application/json", "{\"status\":\"ok\"}");
                                } catch (Exception e) {
                                    sendResponse(output, 400, "application/json",
                                            "{\"status\":\"error\",\"message\":\"" + escapeJson(e.getMessage()) + "\"}");
                                }
                                continue;
                            }

                            // --- POST /run/saved/{pathName} ---
                            if (isPost && "run".equals(action) && pathName.startsWith("saved/")) {
                                String savedName = pathName.substring(6);
                                if (!executionReady) {
                                    sendResponse(output, 503, "application/json",
                                            "{\"status\":\"error\",\"message\":\"No OpMode ready for path execution. Start HttpAuto first.\"}");
                                    continue;
                                }
                                if (pendingExecution) {
                                    sendResponse(output, 409, "application/json",
                                            "{\"status\":\"error\",\"message\":\"A path is already executing\"}");
                                    continue;
                                }
                                String saved = pathCache.get(savedName);
                                if (saved == null) {
                                    sendResponse(output, 404, "application/json",
                                            "{\"status\":\"error\",\"message\":\"Path not found: " + escapeJson(savedName) + "\"}");
                                    continue;
                                }
                                pendingPathJson = saved;
                                pendingExecution = true;
                                Log.i(TAG, "Queued saved path \"" + savedName + "\" for execution");
                                sendResponse(output, 200, "application/json",
                                        "{\"status\":\"ok\",\"path\":\"" + escapeJson(savedName) + "\"}");
                                continue;
                            }

                            // --- POST /run/temp ---
                            if (isPost && "run".equals(action) && "temp".equals(pathName)) {
                                if (!executionReady) {
                                    sendResponse(output, 503, "application/json",
                                            "{\"status\":\"error\",\"message\":\"No OpMode ready for path execution. Start HttpAuto first.\"}");
                                    continue;
                                }
                                if (pendingExecution) {
                                    sendResponse(output, 409, "application/json",
                                            "{\"status\":\"error\",\"message\":\"A path is already executing\"}");
                                    continue;
                                }
                                if (body == null || body.isEmpty()) {
                                    sendResponse(output, 400, "application/json",
                                            "{\"status\":\"error\",\"message\":\"Missing request body with path JSON\"}");
                                    continue;
                                }
                                pendingPathJson = body;
                                pendingExecution = true;
                                Log.i(TAG, "Queued temporary path from request body for execution");
                                sendResponse(output, 200, "application/json",
                                        "{\"status\":\"ok\"}");
                                continue;
                            }

                            sendResponse(output, 405, "text/plain", "Method Not Allowed");
                        } catch (Exception e) {
                            Log.e(TAG, "Request handling error: " + e.getMessage());
                            try {
                                sendResponse(output, 500, "application/json",
                                        "{\"status\":\"error\",\"message\":\"Internal Server Error\"}");
                            } catch (Exception ignored2) {}
                        }
                    }
                } catch (Exception e) {
                    Log.e(TAG, "Error handling connection: " + e.getMessage());
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "Server main loop crashed: " + e.getMessage());
        }
    }

    /** Parse request path, e.g. "POST /save/routeA HTTP/1.1" → "/save/routeA"
     *  URL-decodes percent-encoded UTF-8 characters so Chinese path names work. */
    private static String parseRequestPath(String requestLine) {
        String[] parts = requestLine.split(" ");
        String path = parts.length >= 2 ? parts[1] : "/";
        try {
            path = java.net.URLDecoder.decode(path, String.valueOf(UTF_8));
        } catch (Exception ignored) {}
        return path;
    }

    /**
     * Read one line (terminated by \r\n) from the raw InputStream,
     * decoding as UTF-8. Returns "" for an empty line (just \r\n),
     * or null if the stream ends before any data.
     */
    private static String readLineFromStream(InputStream in) throws IOException {
        ByteArrayOutputStream line = new ByteArrayOutputStream();
        int prev = -1;
        int ch;
        while ((ch = in.read()) != -1) {
            if (prev == '\r' && ch == '\n') {
                // Strip the trailing \r from the accumulated bytes
                byte[] bytes = line.toByteArray();
                int len = bytes.length;
                if (len > 0 && bytes[len - 1] == '\r') {
                    return new String(bytes, 0, len - 1, UTF_8);
                }
                return new String(bytes, UTF_8);
            }
            line.write(ch);
            prev = ch;
        }
        // EOF reached — return whatever was accumulated, or null if nothing
        byte[] bytes = line.toByteArray();
        return bytes.length > 0 ? new String(bytes, UTF_8) : null;
    }

    /**
     * Read exactly {@code contentLength} bytes from the stream.
     * Content-Length is a byte count, not a character count — this is critical
     * for correct UTF-8 handling when the body contains multi-byte characters.
     */
    private static byte[] readBodyBytes(InputStream in, int contentLength) throws IOException {
        byte[] buffer = new byte[contentLength];
        int totalRead = 0;
        while (totalRead < contentLength) {
            int read = in.read(buffer, totalRead, contentLength - totalRead);
            if (read == -1) break;
            totalRead += read;
        }
        return buffer;
    }

    // ---- Persistence: each path saved as a separate SharedPreferences key ----

    private static String prefKey(String pathName) {
        return PREF_KEY_PREFIX + pathName;
    }

    private static void saveToDisk(String pathName, String json) {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(appContext);
        prefs.edit().putString(prefKey(pathName), json).apply();
    }

    private static void deleteFromDisk(String pathName) {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(appContext);
        prefs.edit().remove(prefKey(pathName)).apply();
    }

    /** Load all previously saved paths from SharedPreferences into the in-memory cache */
    private static void loadAllPathsFromDisk() {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(appContext);
        for (String key : prefs.getAll().keySet()) {
            if (key.startsWith(PREF_KEY_PREFIX)) {
                String pathName = key.substring(PREF_KEY_PREFIX.length());
                String json = prefs.getString(key, null);
                if (json != null) {
                    pathCache.put(pathName, json);
                    Log.d(TAG, "Loaded path '" + pathName + "' from disk");
                }
            }
        }
    }

    // ---- HTTP helpers ----

    /**
     * Send an HTTP response with CORS headers.
     * @param contentType may be null (e.g. for 204 No Content)
     * @param body may be null
     */
    private static void sendResponse(OutputStream out, int statusCode,
                                      String contentType, String body) throws Exception {
        byte[] bodyBytes = (body != null) ? body.getBytes(UTF_8) : null;
        StringBuilder header = new StringBuilder();
        String statusText = statusCode == 200 ? "OK"
                : statusCode == 204 ? "No Content"
                : statusCode == 404 ? "Not Found" : "ERROR";
        header.append("HTTP/1.1 ").append(statusCode).append(" ").append(statusText).append("\r\n");
        if (contentType != null) {
            header.append("Content-Type: ").append(contentType).append("\r\n");
        }
        header.append("Access-Control-Allow-Origin: *\r\n");
        header.append("Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n");
        header.append("Access-Control-Allow-Headers: Content-Type\r\n");
        header.append("Access-Control-Max-Age: 86400\r\n");
        if (bodyBytes != null) {
            header.append("Content-Length: ").append(bodyBytes.length).append("\r\n");
        }
        header.append("Connection: close\r\n");
        header.append("\r\n");
        out.write(header.toString().getBytes(UTF_8));
        if (bodyBytes != null) {
            out.write(bodyBytes);
        }
        out.flush();
    }

    // ---- Command registration and invocation ----

    /**
     * Type-level scan: traverses the class hierarchy starting from {@code rootClass},
     * following field types (not values) to discover {@link AutoTask} annotations.
     * Registers command signatures without instances ({@code ready = false}).
     * Called at app startup so command names are visible even before any OpMode runs.
     */
    public static void scanClassTree(Class<?> rootClass) {
        if (rootClass == null) return;
        Log.i(TAG, "Scanning class tree from root: " + rootClass.getSimpleName());

        Deque<Class<?>> stack = new ArrayDeque<>();
        Set<Class<?>> visited = new HashSet<>();
        stack.push(rootClass);

        while (!stack.isEmpty()) {
            Class<?> clazz = stack.pop();
            if (clazz == null || !visited.add(clazz)) continue;

            registerCommandSignatures(clazz);

            if (!isUserClass(clazz)) continue;

            for (Field field : getAllFields(clazz)) {
                if (Modifier.isStatic(field.getModifiers())) continue;
                Class<?> fieldType = field.getType();
                if (fieldType.isPrimitive() || fieldType == String.class
                        || fieldType.isEnum() || Number.class.isAssignableFrom(fieldType)
                        || Boolean.class == fieldType || Character.class == fieldType) {
                    continue;
                }
                stack.push(fieldType);
            }
        }
        Log.i(TAG, "Class scan complete. Pre-registered " + commandRegistry.size() + " command signatures.");
    }

    /** Registers command signatures (name + param types) from a class without an instance. */
    private static void registerCommandSignatures(Class<?> clazz) {
        boolean classAnnotated = clazz.isAnnotationPresent(AutoTask.class);

        for (Method method : clazz.getDeclaredMethods()) {
            AutoTask annot = method.getAnnotation(AutoTask.class);
            if (annot == null && !classAnnotated) continue;
            if (!Modifier.isPublic(method.getModifiers())) continue;

            String commandName = (annot != null && !annot.value().isEmpty())
                    ? annot.value() : method.getName();
            String matchKey = buildMatchKey(clazz, method);

            if (!commandRegistry.containsKey(commandName)) {
                method.setAccessible(true);
                Class<?>[] paramTypes = method.getParameterTypes();
                java.lang.reflect.Parameter[] parameters = method.getParameters();
                String[] paramNames = new String[parameters.length];
                for (int i = 0; i < parameters.length; i++) {
                    paramNames[i] = parameters[i].getName();
                }
                commandRegistry.put(commandName,
                        new CommandEntry(commandName, matchKey, null, method, paramTypes, paramNames, false));
                Log.d(TAG, "Pre-registered command: " + commandName + " (" + clazz.getSimpleName() + ") [not ready]");
            }
        }
    }

    /**
     * Instance-level scan: traverses the object graph starting from {@code root},
     * matching each discovered {@link AutoTask} method against pre-registered entries
     * and populating their instance reference so they become invokable.
     * <p>
     * When a new root is detected (different from previously scanned roots),
     * all existing instances are invalidated first to prevent stale references
     * after an OpMode restart.
     */
    public static void scanObjectTree(Object root) {
        if (root == null) return;
        int rootId = System.identityHashCode(root);
        synchronized (scannedRoots) {
            if (scannedRoots.contains(rootId)) return;
            // New root -> new OpMode, invalidate all old instances
            for (CommandEntry entry : commandRegistry.values()) {
                entry.instance = null;
                entry.ready = false;
            }
            scannedRoots.clear();
            scannedRoots.add(rootId);
        }
        Log.i(TAG, "Scanning object tree from root: " + root.getClass().getSimpleName());

        Deque<Object> stack = new ArrayDeque<>();
        Set<Integer> visited = new HashSet<>();
        stack.push(root);

        while (!stack.isEmpty()) {
            Object obj = stack.pop();
            if (obj == null) continue;
            int id = System.identityHashCode(obj);
            if (!visited.add(id)) continue;

            Class<?> clazz = obj.getClass();
            matchCommandInstances(clazz, obj);

            if (!isUserClass(clazz)) continue;

            for (Field field : getAllFields(clazz)) {
                if (Modifier.isStatic(field.getModifiers())) continue;
                field.setAccessible(true);
                try {
                    Object value = field.get(obj);
                    if (value == null) continue;
                    Class<?> valueClass = value.getClass();
                    if (valueClass.isPrimitive() || valueClass == String.class
                            || valueClass.isEnum() || Number.class.isAssignableFrom(valueClass)
                            || Boolean.class == valueClass || Character.class == valueClass) {
                        continue;
                    }
                    stack.push(value);
                } catch (Exception ignored) {
                }
            }
        }
        Log.i(TAG, "Instance scan complete. " + countReady() + " commands ready.");
    }

    /** Matches a discovered object's methods against pre-registered CommandEntry by matchKey. */
    private static void matchCommandInstances(Class<?> clazz, Object instance) {
        boolean classAnnotated = clazz.isAnnotationPresent(AutoTask.class);

        for (Method method : clazz.getDeclaredMethods()) {
            AutoTask annot = method.getAnnotation(AutoTask.class);
            if (annot == null && !classAnnotated) continue;
            if (!Modifier.isPublic(method.getModifiers())) continue;

            String matchKey = buildMatchKey(clazz, method);
            for (CommandEntry entry : commandRegistry.values()) {
                if (matchKey.equals(entry.matchKey) && entry.instance == null) {
                    entry.method.setAccessible(true);
                    entry.instance = instance;
                    entry.ready = true;
                    Log.d(TAG, "Command ready: " + entry.name + " (" + clazz.getSimpleName() + ")");
                    break;
                }
            }
        }
    }

    /** Builds a unique match key: "fully.qualified.ClassName:methodName:paramType1,paramType2". */
    private static String buildMatchKey(Class<?> clazz, Method method) {
        StringBuilder sb = new StringBuilder(clazz.getName())
                .append(":").append(method.getName()).append(":");
        Class<?>[] params = method.getParameterTypes();
        for (int i = 0; i < params.length; i++) {
            if (i > 0) sb.append(",");
            sb.append(params[i].getName());
        }
        return sb.toString();
    }

    /** Returns the count of ready commands. */
    private static int countReady() {
        int count = 0;
        for (CommandEntry entry : commandRegistry.values()) {
            if (entry.ready) count++;
        }
        return count;
    }

    /** Returns all declared fields of a class, including inherited ones (up to but not including Object). */
    private static List<Field> getAllFields(Class<?> clazz) {
        List<Field> fields = new ArrayList<>();
        while (clazz != null && clazz != Object.class) {
            for (Field f : clazz.getDeclaredFields()) {
                fields.add(f);
            }
            clazz = clazz.getSuperclass();
        }
        return fields;
    }

    /** Returns true if the class belongs to our own codebase (should be recursively traversed). */
    private static boolean isUserClass(Class<?> clazz) {
        String name = clazz.getName();
        return name.startsWith("org.firstinspires.ftc.teamcode");
    }

    /** Returns a human-readable name for a parameter type. */
    private static String typeName(Class<?> type) {
        if (type == int.class) return "int";
        if (type == long.class) return "long";
        if (type == float.class) return "float";
        if (type == double.class) return "double";
        if (type == boolean.class) return "boolean";
        if (type == byte.class) return "byte";
        if (type == short.class) return "short";
        if (type == char.class) return "char";
        if (type == String.class) return "String";
        if (type == Integer.class) return "int";
        if (type == Long.class) return "long";
        if (type == Float.class) return "float";
        if (type == Double.class) return "double";
        if (type == Boolean.class) return "boolean";
        if (type == Byte.class) return "byte";
        if (type == Short.class) return "short";
        if (type == Character.class) return "char";
        return type.getSimpleName();
    }

    /** Converts a JSONArray to an Object[] matching the target parameter types. */
    private static Object[] convertArgs(JSONArray jsonArgs, Class<?>[] paramTypes) throws Exception {
        if (jsonArgs.length() != paramTypes.length) {
            throw new IllegalArgumentException("Expected " + paramTypes.length
                    + " arguments, got " + jsonArgs.length());
        }
        Object[] result = new Object[paramTypes.length];
        for (int i = 0; i < paramTypes.length; i++) {
            result[i] = convertArg(jsonArgs.get(i), paramTypes[i]);
        }
        return result;
    }

    /** Converts a String[] to an Object[] matching the target parameter types. */
    private static Object[] convertStringArgs(String[] stringArgs, Class<?>[] paramTypes) throws Exception {
        if (stringArgs.length != paramTypes.length) {
            throw new IllegalArgumentException("Expected " + paramTypes.length
                    + " arguments, got " + stringArgs.length);
        }
        Object[] result = new Object[paramTypes.length];
        for (int i = 0; i < paramTypes.length; i++) {
            result[i] = convertArg(stringArgs[i], paramTypes[i]);
        }
        return result;
    }

    /** Converts a single JSON value to the target Java type. */
    private static Object convertArg(Object jsonValue, Class<?> targetType) throws Exception {
        if (jsonValue == JSONObject.NULL || jsonValue == null) {
            if (targetType.isPrimitive()) {
                throw new IllegalArgumentException("Cannot pass null for primitive parameter " + typeName(targetType));
            }
            return null;
        }

        if (targetType == String.class) {
            return jsonValue.toString();
        }
        if (targetType == char.class || targetType == Character.class) {
            String s = jsonValue.toString();
            if (s.isEmpty()) throw new IllegalArgumentException("Empty string for char parameter");
            return s.charAt(0);
        }

        // Numeric conversions
        if (jsonValue instanceof Number) {
            Number num = (Number) jsonValue;
            if (targetType == int.class || targetType == Integer.class) return num.intValue();
            if (targetType == long.class || targetType == Long.class) return num.longValue();
            if (targetType == float.class || targetType == Float.class) return num.floatValue();
            if (targetType == double.class || targetType == Double.class) return num.doubleValue();
            if (targetType == byte.class || targetType == Byte.class) return num.byteValue();
            if (targetType == short.class || targetType == Short.class) return num.shortValue();
        }

        if (jsonValue instanceof Boolean) {
            if (targetType == boolean.class || targetType == Boolean.class) return jsonValue;
        }

        // String-to-number fallback
        if (jsonValue instanceof String) {
            String s = (String) jsonValue;
            if (targetType == int.class || targetType == Integer.class) return Integer.parseInt(s);
            if (targetType == long.class || targetType == Long.class) return Long.parseLong(s);
            if (targetType == float.class || targetType == Float.class) return Float.parseFloat(s);
            if (targetType == double.class || targetType == Double.class) return Double.parseDouble(s);
            if (targetType == byte.class || targetType == Byte.class) return Byte.parseByte(s);
            if (targetType == short.class || targetType == Short.class) return Short.parseShort(s);
            if (targetType == boolean.class || targetType == Boolean.class) return Boolean.parseBoolean(s);
            if (targetType == char.class || targetType == Character.class) return s.charAt(0);
        }

        throw new IllegalArgumentException("Cannot convert " + jsonValue.getClass().getSimpleName()
                + " to " + typeName(targetType));
    }

    /** Minimal JSON string escaping. */
    private static String escapeJson(String s) {
        if (s == null) return "";
        return s.replace("\\", "\\\\")
                .replace("\"", "\\\"")
                .replace("\n", "\\n")
                .replace("\r", "\\r")
                .replace("\t", "\\t");
    }

    // ---- Public accessors ----

    /** Returns the current in-memory JSON. */
    public static String getCurrentJson() {
        return currentJson;
    }

    /** Returns the saved JSON for a named path, or null. */
    public static String getSavedJson(String pathName) {
        return pathCache.get(pathName);
    }

    /** Returns all saved path names. */
    public static List<String> getPathNames() {
        return new ArrayList<>(pathCache.keySet());
    }

    /**
     * Triggers the FTC SDK to unregister then re-register all instance OpModes.
     * Called after any save or clear to keep the Driver Station OpMode list
     * in sync with the current path list.
     */
    public static void refreshDynamicOpModes() {
        if (registeredOpModes != null) {
            registeredOpModes.registerInstanceOpModes();
        }
    }

    /**
     * Sets the active Robot instance, enabling live endpoints such as GET /position.
     * Call this from Robot.init() after odometry has been initialized.
     * @param robot the active Robot, or null to clear
     */
    public static void setActiveRobot(Robot robot) {
        activeRobot = robot;
    }

    /**
     * Sets the execution-ready flag. Called by the HttpAuto OpMode when it enters/exits its idle loop.
     * @param ready true when the OpMode is ready to accept path executions, false on stop
     */
    public static void setExecutionReady(boolean ready) {
        executionReady = ready;
    }

    /** Returns true if a path execution is pending (set by HTTP but not yet consumed by OpMode). */
    public static boolean hasPendingExecution() {
        return pendingExecution;
    }

    /** Returns the pending path JSON string (or null). Caller must check hasPendingExecution() first. */
    public static String getPendingPathJson() {
        return pendingPathJson;
    }

    /** Clears the pending execution state. Must be called from the OpMode thread after consuming the JSON. */
    public static void clearPendingExecution() {
        pendingExecution = false;
        pendingPathJson = null;
    }

    /**
     * Sets the name of the currently active OpMode for external visibility.
     * Called by OpModes (HttpAuto, AzConductorDebug, etc.) to identify themselves.
     * @param name the OpMode display name, or null to clear
     */
    public static void setActiveOpModeName(String name) {
        activeOpModeName = name;
    }

    /** Returns the name of the currently active OpMode, or null. */
    public static String getActiveOpModeName() {
        return activeOpModeName;
    }

    /**
     * Creates a Runnable that invokes a registered @AutoTask command by name, passing
     * the given string parameter values. The string values are converted to the
     * correct Java types using {@link #convertStringArgs(String[], Class[])} which
     * delegates to the same {@link #convertArg(Object, Class)} logic used by the
     * HTTP direct-invocation endpoint ({@code POST /commands/run/{name}}).
     *
     * <p>Thread safety: the returned Runnable captures a reference to the
     * {@link CommandEntry} which is stored in a ConcurrentHashMap. The instance
     * reference is stable after {@link #scanObjectTree(Object)} completes, which
     * TrajectoryLoader calls before any command Runnables are fired.</p>
     *
     * @param commandName   the name of the registered @AutoTask command
     * @param commandParams the parameter values as strings; pass an empty array for
     *                      parameterless commands
     * @return a Runnable that invokes the command via reflection, or null if the
     *         command is not found in the registry
     */
    public static Runnable createCommandRunnable(String commandName, String[] commandParams) {
        CommandEntry entry = commandRegistry.get(commandName);
        if (entry == null) {
            Log.w(TAG, "Command not found in registry: " + commandName);
            return null;
        }
        return () -> {
            try {
                if (!entry.ready || entry.instance == null) {
                    Log.w(TAG, "Command not ready at execution time: " + commandName);
                    return;
                }
                if (entry.paramTypes.length > 0) {
                    if (commandParams.length != entry.paramTypes.length) {
                        Log.w(TAG, "Command '" + commandName + "' expects "
                                + entry.paramTypes.length + " arg(s), got "
                                + commandParams.length + ". Skipping.");
                        return;
                    }
                    Object[] args = convertStringArgs(commandParams, entry.paramTypes);
                    entry.method.invoke(entry.instance, args);
                } else {
                    entry.method.invoke(entry.instance);
                }
                Log.i(TAG, "Trajectory executed command: " + commandName);
            } catch (Exception e) {
                Log.e(TAG, "Command invocation failed during trajectory: "
                        + commandName, e);
            }
        };
    }

    /**
     * Convenience overload for parameterless commands. Delegates to
     * {@link #createCommandRunnable(String, String[])} with an empty array.
     *
     * @param commandName the name of the registered @AutoTask command
     * @return a Runnable that invokes the command via reflection, or null if the
     *         command is not found in the registry
     */
    public static Runnable createCommandRunnable(String commandName) {
        return createCommandRunnable(commandName, new String[0]);
    }
}
