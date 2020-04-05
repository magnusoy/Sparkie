import com.github.strikerx3.jxinput.*;
import com.github.strikerx3.jxinput.enums.XInputBatteryDeviceType;
import com.github.strikerx3.jxinput.exceptions.XInputNotLoadedException;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.util.Duration;
import org.controlsfx.control.Notifications;
import org.json.JSONObject;
import xbox.ControllerListener;
import xbox.StorageBox;
import xbox.XboxController;
import java.awt.Desktop;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.lang.reflect.Array;
import java.net.*;
import java.rmi.ConnectIOException;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * Handles all interaction with GUI
 */
public class Controller {

    @FXML TextArea consoleTextArea;                   // Shows Info
    @FXML TextArea xboxConsoleTextArea;               // Shows Xbox data
    private PrintStream console;                      // All info to GUI goes through here

    private ScheduledThreadPoolExecutor scheduler;
    private ScheduledFuture<?> xboxControllerHandler;
    private XboxController xboxController;
    private Client client;                            // Client for sending xbox data to a remote host

    private boolean connectedToController = false;    // xbox controller connection flag
    private boolean connectedToServer = false;        // Server / Remote host connection flag

    // Notifications
    private Notifications serverConnectNotification;
    private Notifications serverDisconnectNotification;

    private Notifications controllerConnectNotification;
    private Notifications controllerDisconnectNotification;

    private String controllerSelectionLabelText = "Xbox Controller Selection";

    /**
     * Everything needed to be done when GUI starts goes here...
     */
    public void initialize() {
        // Create print streams for logging info to GUI consoles
        console = new PrintStream(new Console(consoleTextArea)); // For general information
        // Setup all notifications
        setupNotifications();
        // Stores the initial text from xbox selection header so it can be manipulated later on
        controllerSelectionLabelText = xboxControllerLabel.getText();
        updateNumberOfAvailableControllers();           // Show how many xbox controllers are available at startup
        scheduler = new ScheduledThreadPoolExecutor(3); // Main scheduler for running all tasks with
        console.println("Application Initialized");
    }

    /**
     * Controller listener for receiving xbox controller data (Buttons, axes) and connection statuses.
     *
     * @param storageBox is a shared resource
     */
    private ControllerListener controllerListener = new ControllerListener() {
        @Override
        public void gotData(StorageBox<JSONObject> storageBox) {

            JSONObject jsonObject  = storageBox.get();

            Platform.runLater(() -> {
                // Update xbox console from GUI thread
                xboxConsoleTextArea.setText(jsonObject.toString(2));
                if (connectedToServer){ // Send data to remote server only when connected
                    try {
                        client.send(jsonObject.toString());
                    }catch (Exception e){
                        connectedToServer = false;
                        console.println("Could not reach server.. Please reconnect.");
                        connectToServer(); // Disconnect from server
                    }
                }
            });
        }

        @Override
        public void connected() {
        }

        @Override
        public void disconnected() {
            console.println("Disconnected");
        }
    };

    @FXML Button startVibrationButton;
    private boolean vibration = false;

    /**
     * Toggles the vibration on the connected xbox controller.
     *
     */
    public void toggleVibration(){

        if (connectedToController){// Make sure we are connected to a controller
            if (!vibration){
                xboxController.setVibration(65535, 65535);
                vibration = !vibration;
            }else{
                xboxController.setVibration(0, 0);
                vibration = !vibration;
            }
        }
        // Shows feedback on GUI
        startVibrationButton.setText((vibration? "Stop Vibration" : "Start Vibration"));
    }

    @FXML Label xboxControllerLabel;

    /**
     * Updates the number of available controllers that can be connected to.
     */
    private int updateNumberOfAvailableControllers(){
        int numberOfControllers = 0;

        try {

            XInputDevice14[] devices14 = XInputDevice14.getAllDevices();
            numberOfControllers = Array.getLength(devices14);
            // Don't count the devices within XInputDevice14 that are null
            for (int i = 0; i<4; i++){ // 4 is the maximum number of controllers that can be connected
                if (devices14[i].getBatteryInformation(XInputBatteryDeviceType.GAMEPAD) == null){
                    // Subtract one controller each time info from the controller is not accessible
                    numberOfControllers -= 1;
                }
            }

        } catch (XInputNotLoadedException e) {
            console.println("No Available Controllers");
        }
        controllerConnectButton.setDisable(numberOfControllers == 0);
        xboxControllerLabel.setText(controllerSelectionLabelText + " (" + numberOfControllers
                + (numberOfControllers == 1 ? " available controller)" : " available controllers)"));
        return numberOfControllers;
    }

    @FXML Button controllerRefreshButton;

    /**
     * Called from GUI when controller 'driver' should be refreshed
     */
    public void controllerRefresh(){
        updateNumberOfAvailableControllers();
    }

    @FXML TextField controllerNumberTextField;
    @FXML TextField controllerHandleTextField;
    @FXML Button controllerConnectButton;

    /**
     * Connects to the xBox controller specified in the GUI choice box. If a controller is
     * already connected, a new call will result in disconnection from the connected device.
     */
    public void connectToController() {

        if (!connectedToController){
            try{
                if(updateNumberOfAvailableControllers() > 0){
                    if (!scheduler.isTerminated()){
                        xboxController = new XboxController(0, controllerListener);
                        xboxControllerHandler = scheduler.scheduleAtFixedRate(
                                xboxController, 0, 100, TimeUnit.MILLISECONDS);

                        console.println("Connected to controller " + xboxController.getDeviceNumber());
                        console.println("Battery Level: " + xboxController.getBatteryLevel());

                        controllerConnectNotification.show();
                        connectedToController = !connectedToController;
                    }
                }else console.println("No Controllers Available");


            }catch (ConnectIOException e){
                console.println("No Controllers Available");
            }

        }else {

            xboxController.disconnect();
            xboxControllerHandler.cancel(true); // Cancel the future

            connectedToController = !connectedToController;
            controllerDisconnectNotification.show();
            console.println("Disconnected from controller");
        }
        // Modify GUI elements and enable/disable settings
        controllerNumberTextField.setDisable(connectedToController);
        controllerHandleTextField.setDisable(connectedToController);
        controllerRefreshButton.setDisable(connectedToController);
        controllerConnectButton.setText(connectedToController ? "Disconnect" : "Connect");
        startVibrationButton.setDisable(!connectedToController);
    }

    @FXML TextField serverIPTextField;
    @FXML TextField serverPortTextField;
    @FXML TextField serverHeaderTextField;
    @FXML Button serverConnectButton;

    /**
     * Connects to server with IP and Port from GUI. If the application
     * is already connected, a new call will result in disconnection
     */
    public void connectToServer() {
        if (!connectedToServer){
            // Get ip, port number and header from GUI
            String ip = serverIPTextField.getText();
            int port = Integer.parseInt(serverPortTextField.getText());
            String header = serverHeaderTextField.getText();

            try { // create a client to server connection
                client = new Client(ip, port, header);
                client.start();
                scheduler.submit(client);
                connectedToServer = !connectedToServer;
                serverConnectNotification.show();
                console.println("Connected to server at " + ip + ":" + port);
            }
            catch (ConnectException e){
                console.println("Connect Exception");
            }
            catch (IOException e) {
                console.println("Unable to establish connection with server");
            }
        }else{

            serverDisconnectNotification.show();
            connectedToServer = !connectedToServer;
        }

        serverConnectButton.setText(connectedToServer ? "Disconnect" : "Connect");
        serverIPTextField.setDisable(connectedToServer);
        serverPortTextField.setDisable(connectedToServer);
        serverHeaderTextField.setDisable(connectedToServer);
    }

    /**
     * Clears the console for all text. This also callable from GUI.
     */
    public void clearConsole(){
        consoleTextArea.clear();
    }


    /**
     * Class for redirecting print output stream to GUI console
     */
    public class Console extends OutputStream{
        private TextArea console;

        private Console(TextArea console) {
            this.console = console;
        }

        /**
         * Appends text to the console.
         * @param valueOf the string that should be appended to the console.
         */
        private void appendText(String valueOf){
            Platform.runLater(() -> console.appendText(valueOf));
        }

        public void write(int b){
            appendText(String.valueOf((char)b));
        }
    }

    /**
     * Setup all GUI user notifications with relevant title, text, graphic etc.
     */
    private void setupNotifications(){
        // Set the default position and duration for witch the notifications should appear.
        Pos position = Pos.TOP_RIGHT;
        Duration duration = Duration.seconds(5);


        ImageView img = new ImageView(new Image("petter.jpg"));
        img.setFitHeight(100);
        // img.setPreserveRatio(true);
        img.setFitWidth(80);

        serverConnectNotification = Notifications.create()
                .title("Connected to server")
                .text("Server communication established with " + serverIPTextField.getText()
                        + ":" + serverPortTextField.getText())
                .graphic(img)
                .hideAfter(duration)
                .position(position)
                .darkStyle();

        serverDisconnectNotification = Notifications.create()
                .title("Disconnected from Server")
                .text("Server communication was discontinued")
                .graphic(img)
                .hideAfter(duration)
                .position(position)
                .darkStyle();

        /*serverLostConnectionNotification = Notifications.create()
                .title("Disconnected from Server")
                .text("Lost connection with server")
                .graphic(img)
                .hideAfter(duration)
                .position(position)
                .darkStyle();*/

        /*serverReconnectionNotification = Notifications.create()
                .title("Reconnection to Server")
                .text("Was able to resume communication with: " +
                        serverIPTextField.getText() + ":" + serverPortTextField.getText())
                .graphic(img)
                .hideAfter(duration)
                .position(position)
                .darkStyle();*/

        controllerConnectNotification = Notifications.create()
                .title("Connected to Xbox Controller")
                .text("Connection to Xbox Controller Established")
                .graphic(img)
                .hideAfter(duration)
                .position(position)
                .darkStyle();

        controllerDisconnectNotification = Notifications.create()
                .title("Xbox Controller Disconnected")
                .text("Disconnected from Xbox Controller")
                .graphic(img)
                .hideAfter(duration)
                .position(position)
                .darkStyle();
    }

    /**
     * Opens a github resource for the project.
     */
    public void about(){
        try {
            URL url = new URL("https://github.com/magnusoy/Pick-And-Sort-Robot");
            boolean success = openWebsite(url);
            if (success){
                console.println("Help resource redirected to browser.");
            }
        } catch (MalformedURLException e) {
            e.printStackTrace(); // should never occur unless resource is relocated.
        }

    }

    /**
     * Opens a specific URI in a built-in desktop browser. If for some reason the computer
     * running this does not have a built in browser it will fail and return false.
     *
     * @param uri URI to open
     * @return true when successful in redirecting URI to browser.
     */
    private static boolean openWebsite(URI uri) {
        Desktop desktop = Desktop.isDesktopSupported() ? Desktop.getDesktop() : null;
        if (desktop != null && desktop.isSupported(Desktop.Action.BROWSE)) {
            try {
                desktop.browse(uri);
                return true;
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return false;
    }

    /**
     *
     * @param url URL to open
     * @return true when successful in redirecting URL to browser.
     */
    private static boolean openWebsite(URL url) {
        try {
            return openWebsite(url.toURI());
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
        return false;
    }
}
