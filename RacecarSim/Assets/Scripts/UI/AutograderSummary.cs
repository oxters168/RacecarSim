﻿using System;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

/// <summary>
/// Manages the summary shown after completing the autograder for a lab.
/// </summary>
public class AutograderSummary : MonoBehaviour
{
    #region Set in Unity Editor
    /// <summary>
    /// The UI template which displays the information about a single autograder level.
    /// </summary>
    [SerializeField]
    private GameObject levelEntry;

    /// <summary>
    /// The UI object which contains all of the level entries.
    /// </summary>
    [SerializeField]
    private GameObject levelEntryContainer;
    #endregion

    #region Constants
    /// <summary>
    /// The width of a level entry divided by the space between two level entries.
    /// </summary>
    private const int entryWidthToBufferRatio = 3;

    /// <summary>
    /// The fraction of the container that a single level entry should take up.
    /// </summary>
    private const float entryHeight = 0.2f;

    /// <summary>
    /// The fraction of the container that a level entry should leave unoccupied on the left and right.
    /// </summary>
    private const float entryXBuffer = 0.02f;
    #endregion

    #region Public Interface
    /// <summary>
    /// Return to the main menu.
    /// </summary>
    public void MainMenu()
    {
        SceneManager.LoadScene(LevelCollection.MainMenuBuildIndex, LoadSceneMode.Single);
    }
    #endregion

    /// <summary>
    /// The mutable text fields in the autograder summary, with values corresponding to the index in texts.
    /// </summary>
    private enum Texts
    {
        Title = 0,
        Total = 1
    }

    /// <summary>
    /// The input fields in the autograder summary, with values corresponding to the index in inputFields.
    /// </summary>
    private enum InputFields
    {
        Username = 0,
        ScoreCode = 1
    }

    /// <summary>
    /// The mutable text fields in the autograder summary.
    /// </summary>
    private Text[] texts;

    /// <summary>
    /// The input fields in the autograder summary.
    /// </summary>
    private InputField[] inputFields;

    private void Awake()
    {
        this.texts = this.GetComponentsInChildren<Text>();
        this.inputFields = this.GetComponentsInChildren<InputField>();
    }

    private void Start()
    {
        this.PopulateLevelEntries(LevelManager.LevelInfo.AutograderLevels, AutograderManager.levelScores.ToArray(), out float totalScore, out float totalTime);
        this.texts[(int)Texts.Title].text = $"{LevelManager.LevelInfo.FullName} Autograder";
        this.texts[(int)Texts.Total].text = $"{totalScore:F2}/{LevelManager.LevelInfo.AutograderMaxScore:F2}; {totalTime} seconds";

        this.inputFields[(int)InputFields.Username].text = Settings.Username;
        this.inputFields[(int)InputFields.ScoreCode].text = this.GenerateScoreCode(LevelManager.LevelInfo, totalScore, Settings.Username);
    }

    /// <summary>
    /// Populates the level entry container with information about every level of the autograder.
    /// </summary>
    /// <param name="levelInfos">Information about each autograder level.</param>
    /// <param name="levelScores">Information about the user's performance on each autograder level.</param>
    /// <param name="totalMaxScore">The sum of the max score of each autograder level.</param>
    /// <param name="totalScore">The sum of the user's score on each level.</param>
    /// <param name="totalTime">The sum of the user's time spent on each level.</param>
    private void PopulateLevelEntries(AutograderLevelInfo[] levelInfos, AutograderLevelScore[] levelScores, out float totalScore, out float totalTime)
    {
        totalScore = 0;
        totalTime = 0;

        // Set anchor points of container
        RectTransform container = (RectTransform)this.levelEntryContainer.transform;
        container.anchorMax = new Vector2(1, 1);
        container.anchorMin = new Vector2(0, 1 - AutograderSummary.entryHeight * levelInfos.Length);
        container.anchoredPosition = new Vector2(0, 0);
        container.sizeDelta = new Vector2(0, 0);

        float entryYBuffer = 1.0f / (levelInfos.Length * (AutograderSummary.entryWidthToBufferRatio + 1) + 2);
        float entryHeight = entryYBuffer * AutograderSummary.entryWidthToBufferRatio;
        float anchorY = 1 - entryYBuffer;

        for (int i = 0; i < levelInfos.Length; i++)
        {
            GameObject entry = GameObject.Instantiate(this.levelEntry, Vector3.zero, Quaternion.identity);

            // Set uiEntry's anchor points inside of the container
            entry.transform.SetParent(this.levelEntryContainer.transform);
            RectTransform rect = entry.GetComponent<RectTransform>();
            rect.anchorMax = new Vector2(1 - AutograderSummary.entryXBuffer, anchorY);
            anchorY -= entryHeight;
            rect.anchorMin = new Vector2(AutograderSummary.entryXBuffer, anchorY);
            anchorY -= entryYBuffer;

            // Size exactly to the anchor points
            rect.anchoredPosition = new Vector2(0, 0);
            rect.sizeDelta = new Vector2(0, 0);

            // Set entry
            if (i < levelScores.Length)
            {
                totalScore += levelScores[i].Score;
                totalTime += levelScores[i].Time;
                entry.GetComponent<AutograderUIEntry>().SetInfo(levelInfos[i], levelScores[i]);
            }
            else
            {
                entry.GetComponent<AutograderUIEntry>().SetInfo(levelInfos[i], null);
            }
        }
    }

    /// <summary>
    /// Generates an encrypted score code which encodes the user's score for the level.
    /// </summary>
    /// <param name="levelInfo">Information about the lab.</param>
    /// <param name="score">The user's total score on the lab autograder.</param>
    /// <param name="username">The user's OpenEdx username.</param>
    /// <returns>An hex ciphertext which encodes the user's score for the level.</returns>
    private string GenerateScoreCode(LevelInfo levelInfo, float score, string username)
    {
        string scoreCode = $"{levelInfo.AutograderLevelCode}|{score}|{levelInfo.AutograderMaxScore}|{username}";
        return Utilities.Encrypt(scoreCode);
    }
}
