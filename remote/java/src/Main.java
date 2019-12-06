import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.ButtonType;
import javafx.scene.image.Image;
import javafx.stage.Stage;
import java.util.Optional;

public class Main extends Application {

    public static void main(String[] args){
        launch(args);
    }

    public void start(Stage primaryStage) throws Exception {

        Parent root = FXMLLoader.load(getClass().getResource("xboxClientGUI.fxml"));
        primaryStage.setTitle("Xbox Controller Client");
        primaryStage.setScene(new Scene(root, 800, 600));
        primaryStage.show();
        primaryStage.setIconified(false);
        Image icon = new Image("xboxIcon.png", 225, 0, false, false);
        primaryStage.getIcons().add(icon);
        setUserAgentStylesheet(STYLESHEET_MODENA);
        
        /*
         * Catches and consumes the event where the user presses the built in option to quit the application,
         * and then views a confirmation dialog to the user asking if (s)he really wants to quit.
         */
        primaryStage.setOnCloseRequest(event -> {
            event.consume();
            closeDialog(primaryStage);
            Platform.exit();
            System.exit(0);
        });
    }

    /**
     * Views a close dialog and closes the application if the user confirms (s)he wants to quit.
     *
     * @param primaryStage the Stage to confirm close for.
     */
    private static void closeDialog(Stage primaryStage){
        String title = "Xbox Client";
        String content = "Are you sure you want to quit?";
        String header = "";

        if (confirmBox(title, header, content)){
            primaryStage.close();
        }
    }

    /**
     * Views a confirmation dialog where the user gets a OK / CANCEL choice.
     *
     * @param title the title of the confirmation box
     * @param header the header of the confirmation box
     * @param content the content of the confirmation box
     * @return the boolean answer of the confirmation box
     */
    private static boolean confirmBox(String title, String header, String content) {

        boolean answer = false;

        Alert alert = new Alert(Alert.AlertType.CONFIRMATION);
        alert.setTitle(title);
        alert.setHeaderText(header);
        alert.setContentText(content);

        Optional<ButtonType> result = alert.showAndWait();
        if (result.isPresent()){
            if (result.get() == ButtonType.OK){
                answer = true;
            }
        }
        return answer;
    }
}
