using UnityEngine;
using UnityEngine.UI;

public class SettingsMenu : MonoBehaviour
{
    [Header("Spawner")]
    public SpawnerV2 spawner;

    [Header("Sliders")]
    public Slider amountSlider;
    public Slider intervalSlider;

    [Header("Mass Input")]
    public InputField massInput;

    [Header("UI Text")]
    public Text amountText;
    public Text intervalText;
    public Text massText;

    private void Start()
    {
        if (spawner == null)
        {
            Debug.LogError("SpawnerV2 niet gekoppeld!");
            return;
        }

        if (amountSlider != null)
        {
            amountSlider.value = spawner.spawnAmount;
            amountSlider.onValueChanged.AddListener(UpdateAmount);
        }

        if (intervalSlider != null)
        {
            intervalSlider.value = spawner.spawnInterval;
            intervalSlider.onValueChanged.AddListener(UpdateInterval);
        }

        if (massInput != null)
        {
            massInput.text = spawner.objectMass.ToString();
            massInput.onEndEdit.AddListener(UpdateMass);
        }

        UpdateAmount(spawner.spawnAmount);
        UpdateInterval(spawner.spawnInterval);

        if (massText != null)
        {
            massText.text = "Gewicht: " + spawner.objectMass;
        }

        Debug.Log("SettingsMenu geladen");
    }

    public void UpdateAmount(float value)
    {
        if (spawner == null) return;

        spawner.SetSpawnAmount(value);

        if (amountText != null)
        {
            amountText.text = "Aantal: " + Mathf.RoundToInt(value);
        }
    }

    public void UpdateInterval(float value)
    {
        if (spawner == null) return;

        spawner.SetSpawnInterval(value);

        if (intervalText != null)
        {
            intervalText.text = "Interval: " + value.ToString("F1") + "s";
        }
    }

    public void UpdateMass(string value)
    {
        if (spawner == null) return;

        float mass;

        if (float.TryParse(value, out mass))
        {
            spawner.SetObjectMass(mass);

            if (massText != null)
            {
                massText.text = "Gewicht: " + mass;
            }

            Debug.Log("Massa gewijzigd naar: " + mass);
        }
        else
        {
            Debug.LogWarning("Ongeldige massa ingevoerd: " + value);
        }
    }
}