using UnityEngine;

public class Menu : MonoBehaviour
{
    public GameObject settingsPanel;

    private void Start()
    {
        if (settingsPanel == null)
        {
            Debug.LogError("SettingsPanel is niet gekoppeld!");
        }
    }

    public void ToggleSettings()
    {
        if (settingsPanel == null)
        {
            Debug.LogError("SettingsPanel is null!");
            return;
        }

        bool newState = !settingsPanel.activeSelf;
        settingsPanel.SetActive(newState);

        Debug.Log("ToggleSettings uitgevoerd. SettingsPanel actief: " + newState);
    }

    public void CloseSettings()
    {
        if (settingsPanel == null)
        {
            Debug.LogError("SettingsPanel is null!");
            return;
        }

        settingsPanel.SetActive(false);

        Debug.Log("CloseSettings uitgevoerd. SettingsPanel gesloten.");
    }
}