package xbox;

import org.json.JSONObject;

public interface ControllerListener {
    /**
     * @param jsonBox JSON formatted controller data wrapped in a storage box
     */
    void gotData(StorageBox<JSONObject> jsonBox);

    void connected();

    void disconnected();

}
