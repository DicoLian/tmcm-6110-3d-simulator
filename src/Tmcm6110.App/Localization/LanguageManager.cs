using System.Globalization;
using System.Windows;

namespace Tmcm6110.App.Localization;

/// <summary>
/// Manages runtime localization switching between Traditional Chinese (zh-TW) and English (en-US).
/// Swaps the active ResourceDictionary in Application.Resources.MergedDictionaries for instant UI updates.
/// </summary>
public static class LanguageManager
{
    public const string ZhTw = "zh-TW";
    public const string EnUs = "en-US";

    public static string CurrentLanguage { get; private set; } = ZhTw;
    public static bool IsEnglish => string.Equals(CurrentLanguage, EnUs, StringComparison.OrdinalIgnoreCase);

    public static event Action? LanguageChanged;

    public static void InitializeFromSystemCulture()
    {
        string defaultLang = CultureInfo.CurrentUICulture.Name.StartsWith("zh", StringComparison.OrdinalIgnoreCase)
            ? ZhTw
            : EnUs;
        SetLanguage(defaultLang);
    }

    public static void ToggleLanguage()
    {
        SetLanguage(IsEnglish ? ZhTw : EnUs);
    }

    public static void SetLanguage(string langCode)
    {
        string normalized = string.Equals(langCode, EnUs, StringComparison.OrdinalIgnoreCase) ? EnUs : ZhTw;
        CurrentLanguage = normalized;

        if (Application.Current is { Resources.MergedDictionaries: var merged })
        {
            var newDict = new ResourceDictionary
            {
                Source = new Uri($"pack://application:,,,/Resources/Strings.{normalized}.xaml", UriKind.Absolute)
            };

            int existingIdx = -1;
            for (int i = 0; i < merged.Count; i++)
            {
                string? src = merged[i].Source?.OriginalString;
                if (src != null && src.Contains("Resources/Strings.", StringComparison.OrdinalIgnoreCase))
                {
                    existingIdx = i;
                    break;
                }
            }

            if (existingIdx >= 0)
                merged[existingIdx] = newDict;
            else
                merged.Add(newDict);
        }

        LanguageChanged?.Invoke();
    }

    public static string Get(string zh, string en) => IsEnglish ? en : zh;
}
