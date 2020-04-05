package xbox;
// JXInput is under the MIT licence
import com.github.strikerx3.jxinput.*;
import com.github.strikerx3.jxinput.enums.XInputBatteryDeviceType;
import com.github.strikerx3.jxinput.exceptions.XInputNotLoadedException;
import org.json.JSONObject;
import java.rmi.ConnectIOException;
import java.util.concurrent.locks.ReentrantLock;
import static java.lang.Math.pow;

/**
 * Represents an xbox controller with key data such as button changes and axis values can be subscribed on via
 * a listener interface. Must be scheaduled in a threadpool or similar at a fixed rate that satifies the wanted
 * refresh time for controller keys. For example, the fixed rate can be set to 100 ms and thereby around 5 button
 * clicks per second can be detected.
 */
public class XboxController implements Runnable {

    private Keys keys; // collection of the controller keys
    private static final String[] buttonNames = {"A", "B", "X", "Y", "LEFT_STICK", "LEFT_SHOULDER", "RIGHT_STICK",
            "RIGHT_SHOULDER", "LEFT", "RIGHT", "UP", "DOWN", "START", "BACK"};
    private static final String[] axisNames = {"Left X", "Left Y", "LEFT_TRIGGER", "Right X", "Right Y", "RIGHT_TRIGGER"};
    private StorageBox<JSONObject> jsonBox; // Json in a storage box
    private XInputDevice14 device;
    private XInputAxes axes;
    private XInputButtons buttons;
    private ControllerListener controllerListener;

    private boolean stop = false; // stop flag for polling
    private ReentrantLock lock = new ReentrantLock();
    private int deviceNumber; // Device number for controller


    /**
     * @param deviceNumber the number that identifies the controller.
     * @param listener the subscriber of controller commands
     */
    public XboxController(int deviceNumber, ControllerListener listener) throws ConnectIOException {
        this.deviceNumber = deviceNumber;
        this.controllerListener = listener;
        this.jsonBox = new StorageBox<>();
        this.createKeys(); // Create all xbox keys

        try {
            // Get the device with specified device number
            this.device = XInputDevice14.getDeviceFor(this.deviceNumber);
            this.axes = device.getComponents().getAxes();
            this.buttons = device.getComponents().getButtons();

        } catch (XInputNotLoadedException e) {
            e.printStackTrace();
            throw new ConnectIOException("Connection Failure");
        }
    }

    /**
     * Run method is used to poll device for new data, check for
     * button state changes and hand all the data off to the
     * controller listener in a json format stored in a thread safe
     * container.
     */
    @Override
    public void run() {
        // Polls the device as long as the thread is not stopped
        if (!stop){

            device.poll(); // Query device to update data
            refreshKeys(); // Refresh buttons and axis
            // Create a json dump of the keys
            JSONObject jsonObject = createJSON();
            jsonBox.put(jsonObject);
            // Give new data to the listener
            controllerListener.gotData(jsonBox);
        }
    }

    /**
     * Disconnects the xbox controller. Will stop further processing and remove the device listener.
     */
    public void disconnect(){
        lock.lock(); // locks access to only one thread
        try{
            stop = true;
            controllerListener = null;
        }finally {
            lock.unlock();
        }
    }

    /**
     * Returns the xbox controller battery level.
     * @return the xbox controller battery level as a string
     */
    public String getBatteryLevel(){
        return ("" + device.getBatteryInformation(XInputBatteryDeviceType.GAMEPAD).getLevel());
    }

    /**
     * Controls the right and left vibration actuators on the controller. The level can be set
     * from 0-65535, where zero is off.
     * @param left left side vibration level 0-65535 (0-100%).
     * @param right right side vibration level 0-65535 (0-100%).
     */
    public void setVibration(int left, int right){
        device.setVibration(left, right);
    }

    /**
     * Gets the device number for this xbox controller
     * @return the xbox controller device number
     */
    public int getDeviceNumber() {
        return deviceNumber;
    }

    /**
     * Creates all keys based on the lists of buttons and axes.
     */
    private void createKeys(){
        this.keys = new Keys();
        for (String buttonName : buttonNames) {
            this.keys.addKey(new Button(buttonName));
        }
        for (String axisName : axisNames) {
            this.keys.addKey(new Axis(axisName));
        }
    }

    /**
     * Refresh all Keys (Buttons and Axis) with states and values
     */
    private void refreshKeys(){

        // Update all button states
        for (int i = 0; i < buttonNames.length; i++) {
            Key key = this.keys.getKey(buttonNames[i]);
            if (key instanceof Button){
                Button button = (Button)key;

                switch (i){
                    case 0: button.setState(buttons.a);
                    break;
                    case 1: button.setState(buttons.b);
                        break;
                    case 2: button.setState(buttons.x);
                        break;
                    case 3: button.setState(buttons.y);
                        break;
                    case 4: button.setState(buttons.lThumb);
                        break;
                    case 5: button.setState(buttons.lShoulder);
                        break;
                    case 6: button.setState(buttons.rThumb);
                        break;
                    case 7: button.setState(buttons.rShoulder);
                        break;
                    case 8: button.setState(buttons.left);
                        break;
                    case 9: button.setState(buttons.right);
                        break;
                    case 10: button.setState(buttons.up);
                        break;
                    case 11: button.setState(buttons.down);
                        break;
                    case 12: button.setState(buttons.start);
                        break;
                    case 13: button.setState(buttons.back);
                        break;

                    default: // Do nothing
                }
            }
        }

        // Update all axis values
        for (int i = 0; i < axisNames.length; i++) {
            Key key = keys.getKey(axisNames[i]);
            if (key instanceof Axis) {
                Axis axis = (Axis) key;

                switch(i){
                    case 0: axis.setValue(formatAxisValue(axes.lx));
                    break;
                    case 1: axis.setValue(formatAxisValue(axes.ly));
                        break;
                    case 2: axis.setValue(formatAxisValue(axes.lt));
                        break;
                    case 3: axis.setValue(formatAxisValue(axes.rx));
                        break;
                    case 4: axis.setValue(formatAxisValue(axes.ry));
                        break;
                    case 5: axis.setValue(formatAxisValue(axes.rt));
                        break;

                    default: // Do nothing
                }
            }
        }
    }

    /**
     * Filters and formats a value to its third power and rounds it off to three decimals.
     *
     * @param value value for filtering and formatting
     * @return the filtered and formatted value
     */
    private double formatAxisValue(float value){
        return ((float)((int)(pow(value,3)*1000)))/1000.0;
    }

    /**
     * Creates a JSON file containing all axis and buttons where 'key' is the name of the axis or button, and 'value'
     * is the value or the state. Only buttons that have just changed states are added.
     */
    private JSONObject createJSON(){
        JSONObject jsonObject = new JSONObject();

        for (Key key : keys) {
            if (key instanceof Button){
                if (((Button) key).isStateChanged()){
                    jsonObject.put(key.getName(), ((Button) key).getState());
                }
            }else if(key instanceof Axis){
                jsonObject.put(key.getName(), ((Axis) key).getValue());
            }
        }

        return jsonObject;
    }
}